#pragma once
#include <opencv2/opencv.hpp>

class ITriggerDetector {
public:
    virtual ~ITriggerDetector() = default;
    // Executes lightweight cv::absdiff exclusively on the downrange 3-inch bounding box.
    virtual bool checkOpticalGate(const cv::Mat& currentFrame) = 0;
};
