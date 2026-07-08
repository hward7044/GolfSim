#pragma once
#include "Math/ITriggerDetector.hpp"
#include <opencv2/opencv.hpp>

class OpticalGateTrigger : public ITriggerDetector {
private:
    cv::Rect gateROI;
    cv::Mat  backgroundRef; // Stored as CV_32FC1 for precise EMA updates
    cv::Mat  backgroundRef8U; // 8-bit cached background for fast integer diffs
    cv::Mat  grayRoi;
    cv::Mat  diff8U;
    cv::Mat  thresh;
    cv::Mat  currentRoiFloat;
    int      minBallPixels;
    int      pixelDiffThreshold;
    double   alpha;
public:
    OpticalGateTrigger(
        cv::Rect roi = cv::Rect(400, 300, 200, 200), 
        int minPixels = 150, 
        int threshold = 20, 
        double alphaVal = 0.005
    );

    bool checkOpticalGate(const cv::Mat& currentFrame) override;
    void reset() override;
};

