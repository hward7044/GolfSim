#pragma once
#include <opencv2/core/mat.hpp>
#include "Camera/CameraRole.hpp"
#include <cstdint>
class ICameraNode {
public:
    virtual ~ICameraNode() = default;
    /// @brief Capture a frame into a pre-allocated destination buffer.
    /// @param destination Pre-allocated cv::Mat. Data is memcpy'd in.
    /// @return true if frame was captured successfully.
    virtual bool captureFrame(cv::Mat& destination) = 0;
    virtual CameraRole getRole() = 0;
    /// @brief Timestamp (monotonic microseconds) of the last captured frame, 0 if unavailable.
    virtual uint64_t getLastFrameTimestampUs() const { return 0; }
    virtual void shutdown() {}
};
