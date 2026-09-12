#pragma once
#include <opencv2/core.hpp>

class ITriggerDetector {
public:
    virtual ~ITriggerDetector() = default;
    
    // Evaluates stereoscopic frame pair for ball presence and launch trigger
    virtual bool checkTrigger(const cv::Mat& leftFrame, const cv::Mat& rightFrame) = 0;

    // Legacy single-frame optical gate evaluation (unidirectional helper delegating to checkTrigger)
    bool checkOpticalGate(const cv::Mat& currentFrame) {
        return checkTrigger(currentFrame, currentFrame);
    }

    // Resets background references / tracker state machine to allow clean re-initialization
    virtual void reset() = 0;
};
