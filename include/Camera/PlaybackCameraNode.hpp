#pragma once
#include "Camera/ICameraNode.hpp"
#include <string>
class PlaybackCameraNode : public ICameraNode {
private:
    std::string mockFilePath;
public:
    bool captureFrame(cv::Mat& destination) override;
    CameraRole getRole() override;
};
