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

#ifdef _WIN32
#include "HAL/Win32Serial.hpp"
#endif

// Concrete instantiation of SessionStateMachine for the production hot-path.
// Static dispatch — zero virtual call overhead.
using ConcreteSSM = SessionStateMachine<
    OpticalGateTrigger,
    OpenCVMomentsTracker,
    StereoTriangulator,
    EigenBallisticsEngine,
    TcpJsonTransmitter
>;

#include <thread>
#include <atomic>

class ThreadManager {
private:
    std::shared_ptr<ICameraSystem>          cameraSystem;
    std::shared_ptr<IBufferManager<FrameSet>> buffer;
    std::shared_ptr<ConcreteSSM>            stateMachine;

#ifdef _WIN32
    Win32Serial                             serial_;
#endif

    std::atomic<bool>                       running_{false};
    std::jthread                            producerThread_;
    std::jthread                            consumerThread_;

public:
    ThreadManager() = default;
    ~ThreadManager();

    // Prevent copying
    ThreadManager(const ThreadManager&) = delete;
    ThreadManager& operator=(const ThreadManager&) = delete;

    // Producer Thread: Runs low-power Idle Loop -> trips OpticalGateTrigger ->
    // pushes FrameSets to AtomicRingBuffer.
    void startProducerThread();
    // Consumer Thread: Pops burst FrameSets -> passes to SessionStateMachine ->
    // transmits payload < 15ms.
    void startConsumerThread();
    
    // Stop all threads cleanly
    void stop();
};
