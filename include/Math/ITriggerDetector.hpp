#pragma once
#include <opencv2/core.hpp>

class ITriggerDetector {
public:
    virtual ~ITriggerDetector() = default;
    
    // Evaluates stereoscopic frame pair for ball presence and launch trigger
    virtual bool checkTrigger(const cv::Mat& leftFrame, const cv::Mat& rightFrame) {
        // Default fallback to single-frame check using left frame
        return checkOpticalGate(leftFrame);
    }

    // Legacy single-frame optical gate evaluation (fallback delegating to checkTrigger)
    virtual bool checkOpticalGate(const cv::Mat& currentFrame) {
        return checkTrigger(currentFrame, currentFrame);
    }

    // Resets background references / tracker state machine to allow clean re-initialization
    virtual void reset() = 0;
};
