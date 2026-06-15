#pragma once
#include <opencv2/opencv.hpp>
#include "Camera/CameraRole.hpp"
class ICameraNode {
public:
    virtual ~ICameraNode() = default;
    virtual cv::Mat captureFrame() = 0;
    virtual CameraRole getRole() = 0;
};
