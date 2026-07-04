#include "Math/OpticalGateTrigger.hpp"

OpticalGateTrigger::OpticalGateTrigger(cv::Rect roi, int minPixels, int threshold, double alphaVal)
    : gateROI(roi), minBallPixels(minPixels), pixelDiffThreshold(threshold), alpha(alphaVal) {}

bool OpticalGateTrigger::checkOpticalGate(const cv::Mat& currentFrame) {
    if (currentFrame.empty()) {
        return false;
    }

    // Ensure ROI is within the image bounds
    cv::Rect roi = gateROI & cv::Rect(0, 0, currentFrame.cols, currentFrame.rows);
    if (roi.area() <= 0) {
        return false;
    }

    cv::Mat currentRoi = currentFrame(roi);
    
    // Ensure we work with grayscale
    if (currentRoi.channels() == 3) {
        cv::cvtColor(currentRoi, grayRoi, cv::COLOR_BGR2GRAY);
    } else if (currentRoi.channels() == 4) {
        cv::cvtColor(currentRoi, grayRoi, cv::COLOR_BGRA2GRAY);
    } else {
        grayRoi = currentRoi;
    }

    // Initialize background reference if empty or size changed
    if (backgroundRef.empty() || backgroundRef.size() != roi.size()) {
        grayRoi.convertTo(backgroundRef, CV_32FC1);
        backgroundRef.convertTo(backgroundRef8U, CV_8UC1);
        return false;
    }

    // Compute absolute difference between background and current frame in 8-bit space
    cv::absdiff(grayRoi, backgroundRef8U, diff8U);

    // Threshold the difference image
    cv::threshold(diff8U, thresh, pixelDiffThreshold, 255, cv::THRESH_BINARY);

    // Count changed pixels
    int nonZeroCount = cv::countNonZero(thresh);
    bool triggered = (nonZeroCount >= minBallPixels);

    // Update background reference using EMA, but ONLY for static pixels
    // Static pixels are those where the difference is below the threshold.
    cv::Mat mask = diff8U < pixelDiffThreshold;
    grayRoi.convertTo(currentRoiFloat, CV_32FC1);
    cv::accumulateWeighted(currentRoiFloat, backgroundRef, alpha, mask);
    
    // Synchronize 8-bit background reference cache
    backgroundRef.convertTo(backgroundRef8U, CV_8UC1);

    return triggered;
}
