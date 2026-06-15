#pragma once
#include "FrameSet.hpp"
#include "ITriggerDetector.hpp"
#include "IComputerVision.hpp"
#include "ISpatialSolver.hpp"
#include "IKinematicsSolver.hpp"
#include "INetworkTransmitter.hpp"
#include "FlightRecorder.hpp"
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
