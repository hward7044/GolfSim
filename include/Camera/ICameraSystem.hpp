#pragma once
#include "Camera/FrameSet.hpp"
class ICameraSystem {
public:
    virtual ~ICameraSystem() = default;
    /// @brief Capture synchronized frames into a pre-allocated FrameSet.
    /// @param frameSet Pre-allocated FrameSet with pre-allocated cv::Mat buffers.
    /// @return true if all cameras captured successfully.
    virtual bool captureSynchronizedFrames(FrameSet& frameSet) = 0;
};
