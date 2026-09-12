#pragma once
#include "Math/ITriggerDetector.hpp"
#include "Diagnostics/IDiagnosticProvider.hpp"
#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <chrono>

enum class TriggerState {
    WAITING_FOR_BALL,
    BALL_LOCKED,
    BALL_DEPARTED
};

#include "Math/EmitterPowerMode.hpp"

class BallPresenceTrigger : public ITriggerDetector, public IDiagnosticProvider {
private:
    cv::Rect     teeROI;
    int          lockFrameCount;
    int          ballThreshold;
    double       minBallArea;
    double       maxBallArea;
    double       minCircularity;
    float        matchScoreThreshold;
    double       lossTimeoutSec;

    TriggerState state;
    int          stabilityCounter;
    cv::Point2d  lastCandidateCentroid;
    cv::Rect     lockedBallBox;
    cv::Mat      lockedBallTemplate; // 2D pixel patch of locked ball

    // Emitter protection timer
    std::chrono::steady_clock::time_point emptyStartTime;
    bool             hasEmptyStartTime = false;
    EmitterPowerMode emitterMode = EmitterPowerMode::HIGH_STROBE_READY;

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
        float matchThreshold = 0.45f,
        double lossTimeout = 5.0
    );

    bool checkTrigger(const cv::Mat& leftFrame, const cv::Mat& rightFrame) override;
    bool checkOpticalGate(const cv::Mat& currentFrame);
    void reset() override;

    EmitterPowerMode getEmitterMode() const noexcept { return emitterMode; }
    bool isStandbyRequested() const noexcept { return emitterMode == EmitterPowerMode::LOW_STANDBY; }
    void setLossTimeoutSec(double sec) noexcept { lossTimeoutSec = sec; }

    nlohmann::json getLatestDiagnostics() const override {
        return latestDiag;
    }
};
