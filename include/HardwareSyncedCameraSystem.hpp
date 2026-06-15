#pragma once
#include "ICameraSystem.hpp"
#include "ICameraNode.hpp"
#include <vector>
#include <memory>
class HardwareSyncedCameraSystem : public ICameraSystem {
private:
    std::vector<std::shared_ptr<ICameraNode>> cameras;
public:
    FrameSet captureSynchronizedFrames() override;
};
