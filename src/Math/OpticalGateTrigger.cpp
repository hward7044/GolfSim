#include "Math/OpticalGateTrigger.hpp"

OpticalGateTrigger::OpticalGateTrigger(cv::Rect roi, int minPixels, int threshold, double alphaVal)
    : gateROI(roi), minBallPixels(minPixels), pixelDiffThreshold(threshold), alpha(alphaVal) {}

bool OpticalGateTrigger::checkOpticalGate(const cv::Mat& currentFrame, TriggerDiagnostics* diag) {
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

    // 1. Noise Reduction: Apply Gaussian Blur to smooth pixel jitter
    cv::Mat blurredRoi;
    cv::GaussianBlur(grayRoi, blurredRoi, cv::Size(5, 5), 0);

    // Initialize background reference if empty or size changed
    if (backgroundRef.empty() || backgroundRef.size() != roi.size()) {
        blurredRoi.convertTo(backgroundRef, CV_32FC1);
        backgroundRef.convertTo(backgroundRef8U, CV_8UC1);
        if (diag) {
            diag->triggered = false;
            diag->nonZeroCount = 0;
            diag->minBallPixels = minBallPixels;
            diag->pixelDiffThreshold = pixelDiffThreshold;
            diag->gateROI = roi;
        }
        return false;
    }

    // 2. Global Illumination Correction: Shift brightness to offset AC flicker/auto-exposure
    double meanCurrent = cv::mean(blurredRoi)[0];
    double meanBackground = cv::mean(backgroundRef8U)[0];
    double shift = meanBackground - meanCurrent;

    cv::Mat correctedRoi;
    if (std::abs(shift) > 0.5) {
        // Saturation-cast shift correction directly back into 8U space
        blurredRoi.convertTo(correctedRoi, CV_8UC1, 1.0, shift);
    } else {
        correctedRoi = blurredRoi.clone();
    }

    // 3. Compute absolute difference in corrected space
    cv::absdiff(correctedRoi, backgroundRef8U, diff8U);

    // Threshold the difference image
    cv::threshold(diff8U, thresh, pixelDiffThreshold, 255, cv::THRESH_BINARY);

    // 4. Spatial Consistency: Remove isolated noise specs using Morphological Opening
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);

    // Count changed pixels
    int nonZeroCount = cv::countNonZero(thresh);
    bool triggered = (nonZeroCount >= minBallPixels);

    if (diag) {
        diag->triggered = triggered;
        diag->nonZeroCount = nonZeroCount;
        diag->minBallPixels = minBallPixels;
        diag->pixelDiffThreshold = pixelDiffThreshold;
        diag->gateROI = roi;
    }

    // Update background reference using EMA for static pixels
    // Use the uncorrected blurred frame to track actual slow light drift
    cv::Mat mask = diff8U < pixelDiffThreshold;
    blurredRoi.convertTo(currentRoiFloat, CV_32FC1);
    cv::accumulateWeighted(currentRoiFloat, backgroundRef, alpha, mask);
    
    // Synchronize 8-bit background reference cache
    backgroundRef.convertTo(backgroundRef8U, CV_8UC1);

    return triggered;
}

void OpticalGateTrigger::reset() {
    backgroundRef.release();
    backgroundRef8U.release();
}
