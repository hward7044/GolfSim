#pragma once
#include <opencv2/opencv.hpp>
#include "Camera/CameraRole.hpp"
class ICameraNode {
public:
    virtual ~ICameraNode() = default;
    /// @brief Capture a frame into a pre-allocated destination buffer.
    /// @param destination Pre-allocated cv::Mat. Data is memcpy'd in.
    /// @return true if frame was captured successfully.
    virtual bool captureFrame(cv::Mat& destination) = 0;
    virtual CameraRole getRole() = 0;
};
