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
concept CTriggerDetector = requires(T t, const cv::Mat& f) {
    { t.checkOpticalGate(f) } -> std::same_as<bool>;
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
    std::vector<RecordedFrame> recordedFrames;
    int            emptyFrameCount = 0;
    bool           inShot = false;

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
        network(std::move(n)) {}

    void processNextFrame(const FrameSet& set) {
        cv::Mat leftFrame = set.getFrame(CameraRole::STEREO_LEFT);
        cv::Mat rightFrame = set.getFrame(CameraRole::STEREO_RIGHT);

        if (leftFrame.empty() || rightFrame.empty()) {
            return;
        }

        TriggerDiagnostics trigDiag;
        bool triggeredNow = false;

        // 1. If not currently in a shot, monitor the optical gate for trigger event
        if (!inShot) {
            if (trigger.checkOpticalGate(leftFrame, &trigDiag)) {
                inShot = true;
                trajectoryBuffer.clear();
                recordedFrames.clear();
                emptyFrameCount = 0;
                triggeredNow = true;
                spdlog::info("[SessionStateMachine] Optical gate triggered! Starting shot capture...");
            }
        }

        // 2. If swing is triggered, detect and triangulate coordinates
        if (inShot) {
            VisionDiagnostics leftVisionDiag;
            VisionDiagnostics rightVisionDiag;
            auto leftBalls = vision.detectBalls(leftFrame, &leftVisionDiag);
            auto rightBalls = vision.detectBalls(rightFrame, &rightVisionDiag);

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

            // Buffer the recorded frame (clone frames to avoid sharing references/buffers)
            RecordedFrame rf;
            rf.timestamp = set.timestamp;
            rf.leftFrame = leftFrame.clone();
            rf.rightFrame = rightFrame.clone();
            rf.triggerDiag = trigDiag; // contains valid trigger info if triggered on this frame, otherwise default
            rf.leftVisionDiag = leftVisionDiag;
            rf.rightVisionDiag = rightVisionDiag;
            rf.triangulatedBalls = triangulated;
            recordedFrames.push_back(rf);

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

                    // Save raw and annotated data to FlightRecorder
                    recorder.saveSession(recordedFrames, launchData);
                } else {
                    spdlog::warn("[SessionStateMachine] Trajectory buffer has insufficient points ({}) to solve. Shot discarded.", 
                                 trajectoryBuffer.size());
                }

                // Reset state parameters
                inShot = false;
                trajectoryBuffer.clear();
                recordedFrames.clear();
                emptyFrameCount = 0;
                trigger.reset();
            }
        }
    }
};
