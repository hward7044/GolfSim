#pragma once
#include "ICameraNode.hpp"
#include <string>
class PlaybackCameraNode : public ICameraNode {
private:
    std::string mockFilePath;
public:
    cv::Mat captureFrame() override;
    CameraRole getRole() override;
};
