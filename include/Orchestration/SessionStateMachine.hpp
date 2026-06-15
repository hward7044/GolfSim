#pragma once
#include "Camera/FrameSet.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Diagnostics/FlightRecorder.hpp"
#include <memory>
class SessionStateMachine {
private:
    std::shared_ptr<ITriggerDetector> trigger;
    std::shared_ptr<IComputerVision> vision;
    std::shared_ptr<ISpatialSolver> spatial;
    std::shared_ptr<IKinematicsSolver> kinematics;
    std::shared_ptr<INetworkTransmitter> network;
    std::shared_ptr<FlightRecorder> recorder;
public:
    void processNextFrame(const FrameSet& set);
};
