#pragma once
#include "Math/ITriggerDetector.hpp"
#include "Diagnostics/IDiagnosticProvider.hpp"
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

enum class TriggerState {
    WAITING_FOR_BALL,
    BALL_LOCKED,
    BALL_DEPARTED
};

class BallPresenceTrigger : public ITriggerDetector, public IDiagnosticProvider {
private:
    cv::Rect     teeROI;
    int          lockFrameCount;
    int          ballThreshold;
    double       minBallArea;
    double       maxBallArea;
    double       minCircularity;
    float        matchScoreThreshold;

    TriggerState state;
    int          stabilityCounter;
    cv::Point2d  lastCandidateCentroid;
    cv::Rect     lockedBallBox;
    cv::Mat      lockedBallTemplate; // 2D pixel patch of locked ball

    // Scratchpad variables for zero-allocation hot path
    cv::Mat      grayRoi;
    cv::Mat      threshRoi;
    cv::Mat      matchResult;

    nlohmann::json latestDiag;

public:
    BallPresenceTrigger(
        cv::Rect roi = cv::Rect(400, 460, 160, 160),
        int lockFrames = 30,
        int thresh = 120,
        double minArea = 80.0,
        double maxArea = 2500.0,
        double minCirc = 0.65,
        float matchThreshold = 0.45f
    );

    bool checkOpticalGate(const cv::Mat& currentFrame) override;
    void reset() override;

    nlohmann::json getLatestDiagnostics() const override {
        return latestDiag;
    }
};
