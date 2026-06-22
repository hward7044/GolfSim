#include "Camera/HardwareSyncedCameraSystem.hpp"

bool HardwareSyncedCameraSystem::captureSynchronizedFrames(FrameSet& frameSet) {
    // Iterate cameras, write each frame into the FrameSet's pre-allocated slot
    for (auto& camera : cameras) {
        CameraRole role = camera->getRole();
        if (!camera->captureFrame(frameSet.getFrame(role))) {
            return false;
        }
    }
    return true;
}
