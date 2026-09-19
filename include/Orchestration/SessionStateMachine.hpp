#pragma once
#include "Camera/FrameSet.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Diagnostics/FlightRecorder.hpp"
#include "Orchestration/PipelineTimingConfig.hpp"

#include <vector>
#include <fstream>
#include <chrono>
#include <functional>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

// T.2 — algorithms for many argument types
// T.5 — combine generic + OO techniques
//
// Static dispatch on hot path; concept constraints preserve interface
// contracts for testing.
//
// template<ITriggerDetector  Trigger,
//          IComputerVision   Vision,
//          ISpatialSolver    Spatial,
//          IKinematicsSolver Kinematics,
//          INetworkTransmitter Net>

// --- C++20 Concept definitions (mirror the interface contracts) ---
template<typename T>
concept CTriggerDetector = requires(T t, const cv::Mat& f1, const cv::Mat& f2) {
    { t.checkTrigger(f1, f2) } -> std::same_as<bool>;
    t.reset();
};

template<typename T>
concept CComputerVision = std::derived_from<T, IComputerVision>;

template<typename T>
concept CSpatialSolver = std::derived_from<T, ISpatialSolver>;

template<typename T>
concept CKinematicsSolver = std::derived_from<T, IKinematicsSolver>;

template<typename T>
concept CNetworkTransmitter = std::derived_from<T, INetworkTransmitter>;

// --- Templated State Machine ---
template<
    CTriggerDetector    Trigger,
    CComputerVision     Vision,
    CSpatialSolver      Spatial,
    CKinematicsSolver   Kinematics,
    CNetworkTransmitter Net
>
class SessionStateMachine {
private:
    Trigger        trigger;
    Vision         vision;
    Spatial        spatial;
    Kinematics     kinematics;
    Net            network;
    FlightRecorder recorder;
    std::string    shotHistoryPath_;

    // Timing and optical configuration
    PipelineTimingConfig timingConfig;

    // Track state across incoming frames
    std::vector<Ball3D> trajectoryBuffer;
    std::vector<RecordedFrame> recordedFramesPool;
    size_t         recordedFrameCount = 0;
    int            emptyFrameCount = 0;
    int            shotFrameCount = 0;
    bool           inShot = false;

    bool           streamRecordingMode = false;
    int            streamFrameLimit = 50;
    std::vector<RecordedFrame> streamFramesPool;
    size_t         streamFrameCount = 0;

    std::function<void(char)> onSerialCommand_ = nullptr;
    char           lastCommandSent_ = 0;

    void sendSerialCommand(char cmd) {
        if (onSerialCommand_ && cmd != lastCommandSent_) {
            onSerialCommand_(cmd);
            lastCommandSent_ = cmd;
        }
    }

    template<typename T>
    nlohmann::json getTelemetry(const T& obj) {
        if constexpr (requires { obj.getLatestDiagnostics(); }) {
            return obj.getLatestDiagnostics();
        }
        return nlohmann::json::object();
    }

    // Helper to append solved shot data to the JSON Lines history file
    void saveToShotHistory(const LaunchData<Degrees, MilesPerHour>& data) {
        nlohmann::json j;
        j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        j["ballSpeed_mph"] = data.ballSpeed.value();
        j["verticalLaunchAngle_deg"] = data.verticalLaunchAngle.value();
        j["horizontalLaunchAngle_deg"] = data.horizontalLaunchAngle.value();
        j["spinRPM"] = data.spinRPM;
        j["spinAxis"] = { data.spinAxis.x(), data.spinAxis.y(), data.spinAxis.z() };

        // Append to JSON Lines file for easy programmatic parsing
        std::ofstream out(shotHistoryPath_, std::ios::app);
        if (out.is_open()) {
            out << j.dump() << "\n";
        } else {
            spdlog::warn("[SessionStateMachine] Could not open shot history file {}", shotHistoryPath_);
        }
    }

public:
    SessionStateMachine(
        Trigger t = Trigger(),
        Vision v = Vision(),
        Spatial s = Spatial(),
        Kinematics k = Kinematics(),
        Net n = Net(),
        PipelineTimingConfig timing = PipelineTimingConfig(),
        std::string replayDir = "build/replays",
        std::string shotHistoryPath = "build/shot_history.json"
    ) : trigger(std::move(t)),
        vision(std::move(v)),
        spatial(std::move(s)),
        kinematics(std::move(k)),
        network(std::move(n)),
        recorder(replayDir),
        shotHistoryPath_(std::move(shotHistoryPath)),
        timingConfig(timing) {
        // Pre-allocate 40 recorded frames for zero-allocation copying in the hot path
        recordedFramesPool.resize(40);
        for (auto& rf : recordedFramesPool) {
            rf.leftFrame = cv::Mat(800, 1280, CV_8UC1);
            rf.rightFrame = cv::Mat(800, 1280, CV_8UC1);
        }
    }

    const PipelineTimingConfig& getTimingConfig() const { return timingConfig; }
    void setTimingConfig(const PipelineTimingConfig& config) { timingConfig = config; }

    void setStreamRecordingMode(bool enable, int frameLimit = 50) {
        streamRecordingMode = enable;
        streamFrameLimit = frameLimit;
        streamFramesPool.resize(frameLimit);
        for (auto& rf : streamFramesPool) {
            rf.leftFrame = cv::Mat(800, 1280, CV_8UC1);
            rf.rightFrame = cv::Mat(800, 1280, CV_8UC1);
        }
        streamFrameCount = 0;
        spdlog::info("[SessionStateMachine] Stream recording mode {} (Limit: {} frames/chunk)",
                     enable ? "ENABLED" : "DISABLED", frameLimit);
    }

    void setSerialCallback(std::function<void(char)> cb) {
        onSerialCommand_ = std::move(cb);
    }

    void processNextFrame(const FrameSet& set) {
        cv::Mat leftFrame = set.getFrame(CameraRole::STEREO_LEFT);
        cv::Mat rightFrame = set.getFrame(CameraRole::STEREO_RIGHT);

        if (leftFrame.empty() || rightFrame.empty()) {
            return;
        }

        nlohmann::json trigDiag;

        // If in stream recording mode, buffer frames and write stream chunks to disk without waiting for a shot trigger
        // Stream recording mode bypasses shot state machine
        if (streamRecordingMode) {
            if (streamFrameCount < streamFramesPool.size()) {
                auto& rf = streamFramesPool[streamFrameCount++];
                rf.timestamp = set.timestamp;
                leftFrame.copyTo(rf.leftFrame);
                rightFrame.copyTo(rf.rightFrame);
            }

            if (streamFrameCount >= streamFrameLimit) {
                std::vector<RecordedFrame> streamFrames(streamFramesPool.begin(), streamFramesPool.begin() + streamFrameCount);
                recorder.saveStreamSession(streamFrames);
                streamFrameCount = 0;
            }
            return;
        }

        // 1. If not currently in a shot, monitor the trigger detector for trigger event
        if (!inShot) {
            if (trigger.checkTrigger(leftFrame, rightFrame)) {
                inShot = true;
                trajectoryBuffer.clear();
                recordedFrameCount = 0;
                emptyFrameCount = 0;
                shotFrameCount = 0;
                trigDiag = getTelemetry(trigger);
                spdlog::info("[SessionStateMachine] Impact trigger confirmed! Starting shot capture...");
                sendSerialCommand('H');
            } else {
                if constexpr (requires { trigger.isStandbyRequested(); }) {
                    if (trigger.isStandbyRequested()) {
                        sendSerialCommand('L');
                    } else {
                        sendSerialCommand('H');
                    }
                }
            }
        }

        // 2. If swing is triggered, detect and triangulate coordinates
        if (inShot) {
            shotFrameCount++;

            auto leftBalls = vision.detectBalls(leftFrame);
            nlohmann::json leftVisionDiag = getTelemetry(vision);

            auto rightBalls = vision.detectBalls(rightFrame);
            nlohmann::json rightVisionDiag = getTelemetry(vision);

            std::vector<Ball3D> triangulated;
            if (!leftBalls.empty() || !rightBalls.empty()) {
                triangulated = spatial.triangulateShot(leftBalls, rightBalls);

                if (!triangulated.empty()) {
                    trajectoryBuffer.insert(trajectoryBuffer.end(), triangulated.begin(), triangulated.end());
                    emptyFrameCount = 0;
                } else {
                    emptyFrameCount++;
                }
            } else {
                emptyFrameCount++;
            }

            // Buffer the recorded frame (zero allocations memcpy)
            if (recordedFrameCount < recordedFramesPool.size()) {
                auto& rf = recordedFramesPool[recordedFrameCount];
                rf.timestamp = set.timestamp;
                leftFrame.copyTo(rf.leftFrame);
                rightFrame.copyTo(rf.rightFrame);
                rf.triggerDiag = trigDiag;
                rf.leftVisionDiag = leftVisionDiag;
                rf.rightVisionDiag = rightVisionDiag;
                rf.triangulatedBalls = triangulated;
                recordedFrameCount++;
            }

            // 3. Low-Latency 1-to-2 Frame Hybrid Completion Check:
            //    - Ball exited: Seen pulses previously and consecutive empty frames >= emptyFrameTimeout (default: 1)
            //    - Frame limit reached: Captured maxFramesPerShot (default: 2)
            //    - Point buffer full: Accumulated maxFramesPerShot * 5 pulses
            bool ballExited = (!trajectoryBuffer.empty() && emptyFrameCount >= timingConfig.emptyFrameTimeout);
            bool frameLimitReached = (shotFrameCount >= timingConfig.maxFramesPerShot);
            bool pointBufferFull = (trajectoryBuffer.size() >= static_cast<size_t>(timingConfig.maxFramesPerShot * 5));

            if (ballExited || frameLimitReached || pointBufferFull) {
                spdlog::info("[SessionStateMachine] Shot capture completed. Buffered points: {}, Shot frames: {}, Empty frames: {}", 
                             trajectoryBuffer.size(), shotFrameCount, emptyFrameCount);

                if (trajectoryBuffer.size() >= static_cast<size_t>(timingConfig.minPointsToSolve)) {
                    spdlog::info("[SessionStateMachine] Solving shot kinematics (pulseInterval: {:.2f} ms)...",
                                 timingConfig.pulseIntervalMs);
                    LaunchData<Degrees, MilesPerHour> launchData = 
                        kinematics.solveKinematics(trajectoryBuffer, timingConfig.pulseIntervalMs);

                    spdlog::info("[SessionStateMachine] Shot Solved: Speed={:.1f} mph | VLA={:.1f} deg | HLA={:.1f} deg | Spin={:.0f} RPM",
                                 launchData.ballSpeed.value(), launchData.verticalLaunchAngle.value(),
                                 launchData.horizontalLaunchAngle.value(), launchData.spinRPM);

                    // Transmit JSON payload to clients
                    network.transmitLaunchData(launchData);

                    // Save shot parameters to file
                    saveToShotHistory(launchData);

                    // Save raw and annotated data to FlightRecorder asynchronously
                    std::vector<RecordedFrame> activeFrames(recordedFramesPool.begin(), recordedFramesPool.begin() + recordedFrameCount);
                    recorder.saveSession(activeFrames, launchData);
                } else {
                    spdlog::warn("[SessionStateMachine] Trajectory buffer has insufficient points ({} < {}) to solve. Shot discarded.", 
                                 trajectoryBuffer.size(), timingConfig.minPointsToSolve);
                }

                // Reset state parameters
                inShot = false;
                trajectoryBuffer.clear();
                recordedFrameCount = 0;
                emptyFrameCount = 0;
                shotFrameCount = 0;
                trigger.reset();
            }
        }
    }
};
