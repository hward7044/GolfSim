#include "Orchestration/ThreadManager.hpp"
#include <spdlog/spdlog.h>
#include <chrono>

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::startProducerThread() {
    if (running_.exchange(true)) {
        spdlog::warn("[ThreadManager] Threads are already running or startup is in progress.");
        return;
    }

    producerThread_ = std::jthread([this](std::stop_token stopToken) {
        spdlog::info("[ThreadManager] Starting Producer Thread...");

#ifdef _WIN32
        // Open the serial port for the MCU trigger board once at session startup.
        // COM3 is used as a default placeholder; this can be loaded from configuration.
        std::string comPort = "COM3"; 
        int baudRate = 115200;
        if (!serial_.open(comPort, baudRate)) {
            spdlog::error("[ThreadManager] Trigger MCU serial port configuration failed. "
                          "Launch monitors will not be triggered automatically.");
        }
#endif

        FrameSet frameSet;
        // Pre-allocate frame buffers (OV9281 resolution: 1280x800)
        frameSet.preallocate(1280, 800);

        // Optical gate trigger helper for idle detection
        OpticalGateTrigger gateTrigger;

        while (!stopToken.stop_requested() && running_) {
            if (!cameraSystem) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Capture frames continuously from the always-triggered cameras
            if (cameraSystem->captureSynchronizedFrames(frameSet)) {
                // Buffer the captured FrameSet
                if (buffer) {
                    buffer->push(frameSet);
                }

                // Check optical gate trigger using the left camera frame
                cv::Mat leftFrame = frameSet.getFrame(CameraRole::STEREO_LEFT);
                if (!leftFrame.empty()) {
                    if (gateTrigger.checkOpticalGate(leftFrame)) {
                        spdlog::info("[ThreadManager] Optical gate triggered! Dispatching MCU serial FIRE signal.");
#ifdef _WIN32
                        if (serial_.isOpen()) {
                            serial_.writeString("FIRE\n");
                        }
#endif
                    }
                }
            } else {
                // If frame capture is waiting on external triggers (MCU low-rate idle),
                // sleep briefly to prevent CPU spinning if there's a timeout.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

#ifdef _WIN32
        serial_.close();
#endif
        spdlog::info("[ThreadManager] Producer Thread terminated cleanly.");
    });
}

void ThreadManager::startConsumerThread() {
    if (!running_) {
        spdlog::warn("[ThreadManager] Cannot start consumer thread: manager is not in a running state.");
        return;
    }

    consumerThread_ = std::jthread([this](std::stop_token stopToken) {
        spdlog::info("[ThreadManager] Starting Consumer Thread...");

        while (!stopToken.stop_requested() && running_) {
            if (!buffer || !stateMachine) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            auto frameSetOpt = buffer->pop();
            if (frameSetOpt) {
                // Process the frames in the mathematical/vision pipeline
                stateMachine->processNextFrame(*frameSetOpt);
            } else {
                // No frames available in the atomic ring buffer; yield execution
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        spdlog::info("[ThreadManager] Consumer Thread terminated cleanly.");
    });
}

void ThreadManager::stop() {
    if (!running_.exchange(false)) {
        return; // Already stopped
    }

    spdlog::info("[ThreadManager] Initiating ThreadManager stop sequence.");

    // 1. First shut down the camera system to unblock any ReadSample blocks in the producer thread
    if (cameraSystem) {
        cameraSystem->shutdown();
    }

    // 2. Signal stop tokens to threads
    producerThread_.request_stop();
    consumerThread_.request_stop();

    // 3. Join the threads
    if (producerThread_.joinable()) {
        producerThread_.join();
    }
    if (consumerThread_.joinable()) {
        consumerThread_.join();
    }

    spdlog::info("[ThreadManager] ThreadManager stopped successfully.");
}
