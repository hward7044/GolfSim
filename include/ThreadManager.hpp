#pragma once
#include "ICameraSystem.hpp"
#include "IBufferManager.hpp"
#include "SessionStateMachine.hpp"
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
