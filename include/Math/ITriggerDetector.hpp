#pragma once
#include <opencv2/core.hpp>

class ITriggerDetector {
public:
    virtual ~ITriggerDetector() = default;
    // Executes lightweight cv::absdiff exclusively on the downrange 3-inch bounding box.
    virtual bool checkOpticalGate(const cv::Mat& currentFrame) = 0;
    // Resets background references to allow clean re-initialization
    virtual void reset() = 0;
};
