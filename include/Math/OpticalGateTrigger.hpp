#pragma once
#include "Math/ITriggerDetector.hpp"
#include <opencv2/opencv.hpp>

class OpticalGateTrigger : public ITriggerDetector {
private:
    // Bounding box of the downrange 3-inch optical gate region
    cv::Rect gateROI;
    // Reference background frame for differencing
    cv::Mat  backgroundRef;
public:
    // Executes lightweight cv::absdiff exclusively on gateROI to detect ball passage.
    bool checkOpticalGate(const cv::Mat& currentFrame) override;
};
