#pragma once
#include "Camera/ICameraSystem.hpp"
#include "Math/IBufferManager.hpp"
#include "Orchestration/SessionStateMachine.hpp"
#include <memory>
class ThreadManager {
private:
    std::shared_ptr<ICameraSystem> cameraSystem;
    std::shared_ptr<IBufferManager> buffer;
    std::shared_ptr<SessionStateMachine> stateMachine;
public:
    void startProducerThread();
    void startConsumerThread();
};
