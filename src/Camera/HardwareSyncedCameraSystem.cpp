#include "Camera/HardwareSyncedCameraSystem.hpp"

bool HardwareSyncedCameraSystem::captureSynchronizedFrames(FrameSet& frameSet) {
    if (cameras.empty()) {
        return false;
    }
    // Iterate cameras, write each frame into the FrameSet's pre-allocated slot
    for (auto& camera : cameras) {
        CameraRole role = camera->getRole();
        if (!camera->captureFrame(frameSet.getFrame(role))) {
            return false;
        }
    }
    return true;
}

void HardwareSyncedCameraSystem::addCameraNode(std::shared_ptr<ICameraNode> camera) {
    if (camera) {
        cameras.push_back(camera);
    }
}

void HardwareSyncedCameraSystem::shutdown() {
    for (auto& camera : cameras) {
        camera->shutdown();
    }
}
