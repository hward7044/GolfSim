#pragma once
#include <opencv2/opencv.hpp>

struct TriggerDiagnostics {
    int nonZeroCount = 0;
    int minBallPixels = 0;
    int pixelDiffThreshold = 0;
    bool triggered = false;
    cv::Rect gateROI;
};

class ITriggerDetector {
public:
    virtual ~ITriggerDetector() = default;
    // Executes lightweight cv::absdiff exclusively on the downrange 3-inch bounding box.
    virtual bool checkOpticalGate(const cv::Mat& currentFrame) = 0;
    virtual bool checkOpticalGate(const cv::Mat& currentFrame, TriggerDiagnostics* diag) {
        bool res = checkOpticalGate(currentFrame);
        if (diag) {
            diag->triggered = res;
            diag->nonZeroCount = res ? 1000 : 0; // fallback placeholders
            diag->minBallPixels = 100;
            diag->pixelDiffThreshold = 20;
            diag->gateROI = cv::Rect();
        }
        return res;
    }
    // Resets background references to allow clean re-initialization
    virtual void reset() = 0;
};
