#pragma once
#include "Camera/FrameSet.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Diagnostics/FlightRecorder.hpp"

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
public:
    void processNextFrame(const FrameSet& set) {
        // Stub: check trigger, run vision/spatial/kinematics pipeline, transmit
    }
};
