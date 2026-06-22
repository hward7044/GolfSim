#pragma once
#include "Camera/ICameraSystem.hpp"
#include "Camera/ICameraNode.hpp"
#include <vector>
#include <memory>
class HardwareSyncedCameraSystem : public ICameraSystem {
private:
    std::vector<std::shared_ptr<ICameraNode>> cameras;
public:
    bool captureSynchronizedFrames(FrameSet& frameSet) override;
};
