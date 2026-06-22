#pragma once
#include "Camera/ICameraSystem.hpp"
#include "Math/IBufferManager.hpp"
#include "Math/OpticalGateTrigger.hpp"
#include "Math/OpenCVMomentsTracker.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/EigenBallisticsEngine.hpp"
#include "Math/TcpJsonTransmitter.hpp"
#include "Orchestration/SessionStateMachine.hpp"
#include <memory>

// Concrete instantiation of SessionStateMachine for the production hot-path.
// Static dispatch — zero virtual call overhead.
using ConcreteSSM = SessionStateMachine<
    OpticalGateTrigger,
    OpenCVMomentsTracker,
    StereoTriangulator,
    EigenBallisticsEngine,
    TcpJsonTransmitter
>;

class ThreadManager {
private:
    std::shared_ptr<ICameraSystem>          cameraSystem;
    std::shared_ptr<IBufferManager<FrameSet>> buffer;
    std::shared_ptr<ConcreteSSM>            stateMachine;
public:
    // Producer Thread: Runs low-power Idle Loop -> trips OpticalGateTrigger ->
    // injects I2C Strobe command -> pushes rapid burst of FrameSets to AtomicRingBuffer.
    void startProducerThread();
    // Consumer Thread: Pops burst FrameSets -> passes to SessionStateMachine ->
    // transmits payload < 15ms.
    void startConsumerThread();
};
