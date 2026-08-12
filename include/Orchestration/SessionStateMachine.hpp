#pragma once
#include "Camera/FrameSet.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Diagnostics/FlightRecorder.hpp"

#include <vector>
#include <fstream>
#include <chrono>
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

    // Track state across incoming frames
    std::vector<Ball3D> trajectoryBuffer;
    std::vector<RecordedFrame> recordedFramesPool;
    size_t         recordedFrameCount = 0;
    int            emptyFrameCount = 0;
    bool           inShot = false;

    bool           streamRecordingMode = false;
    int            streamFrameLimit = 50;
    std::vector<RecordedFrame> streamFramesPool;
    size_t         streamFrameCount = 0;

    template<typename T>
    nlohmann::json getTelemetry(const T& obj) {
        if constexpr (requires { obj.getLatestDiagnostics(); }) {
            return obj.getLatestDiagnostics();
        }
        return nlohmann::json::object();
    }

    // Helper to log solved shot data to build/shot_history.json
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
        std::ofstream out("build/shot_history.json", std::ios::app);
        if (out.is_open()) {
            out << j.dump() << "\n";
            out.close();
        } else {
            std::ofstream outFallback("shot_history.json", std::ios::app);
            if (outFallback.is_open()) {
                outFallback << j.dump() << "\n";
                outFallback.close();
            }
        }
    }

public:
    SessionStateMachine(
        Trigger t = Trigger(),
        Vision v = Vision(),
        Spatial s = Spatial(),
        Kinematics k = Kinematics(),
        Net n = Net()
    ) : trigger(std::move(t)),
        vision(std::move(v)),
        spatial(std::move(s)),
        kinematics(std::move(k)),
        network(std::move(n)) {
        // Pre-allocate 40 recorded frames for zero-allocation copying in the hot path
        recordedFramesPool.resize(40);
        for (auto& rf : recordedFramesPool) {
            rf.leftFrame = cv::Mat(800, 1280, CV_8UC1);
            rf.rightFrame = cv::Mat(800, 1280, CV_8UC1);
        }
    }

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

    void processNextFrame(const FrameSet& set) {
        cv::Mat leftFrame = set.getFrame(CameraRole::STEREO_LEFT);
        cv::Mat rightFrame = set.getFrame(CameraRole::STEREO_RIGHT);

        if (leftFrame.empty() || rightFrame.empty()) {
            return;
        }

        nlohmann::json trigDiag;

        // If in stream recording mode, buffer frames and write stream chunks to disk without waiting for a shot trigger
        if (streamRecordingMode) {
            trigger.checkTrigger(leftFrame, rightFrame);
            trigDiag = getTelemetry(trigger);

            if (streamFrameCount < streamFramesPool.size()) {
                auto& rf = streamFramesPool[streamFrameCount];
                rf.timestamp = set.timestamp;
                leftFrame.copyTo(rf.leftFrame);
                rightFrame.copyTo(rf.rightFrame);
                rf.triggerDiag = trigDiag;
                streamFrameCount++;
            }

            if (streamFrameCount >= static_cast<size_t>(streamFrameLimit)) {
                spdlog::info("[SessionStateMachine] Stream chunk captured ({}/{} frames). Saving to disk...",
                             streamFrameCount, streamFrameLimit);
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
                trigDiag = getTelemetry(trigger);
                spdlog::info("[SessionStateMachine] Impact trigger confirmed! Starting shot capture...");
            }
        }

        // 2. If swing is triggered, detect and triangulate coordinates
        if (inShot) {
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

            // 3. Option C Completion Check:
            //    - emptyFrameCount >= 15 (ball has left the frame) OR
            //    - trajectoryBuffer.size() >= 25 (safety cutoff)
            if (emptyFrameCount >= 15 || trajectoryBuffer.size() >= 25) {
                spdlog::info("[SessionStateMachine] Shot capture completed. Buffered points: {}, Empty frames: {}", 
                             trajectoryBuffer.size(), emptyFrameCount);

                if (trajectoryBuffer.size() >= 3) {
                    spdlog::info("[SessionStateMachine] Solving shot kinematics...");
                    // Strobe interval is 1.0ms
                    LaunchData<Degrees, MilesPerHour> launchData = kinematics.solveKinematics(trajectoryBuffer, 1.0);

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
                    spdlog::warn("[SessionStateMachine] Trajectory buffer has insufficient points ({}) to solve. Shot discarded.", 
                                 trajectoryBuffer.size());
                }

                // Reset state parameters
                inShot = false;
                trajectoryBuffer.clear();
                recordedFrameCount = 0;
                emptyFrameCount = 0;
                trigger.reset();
            }
        }
    }
};
