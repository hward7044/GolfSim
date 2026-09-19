#pragma once
#include "Math/ITriggerDetector.hpp"
#include "Math/DotClusterFinder.hpp"
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

/// Single-camera presence trigger over the dot-cluster detector: locks when
/// exactly one dot cluster sits still for `lockFrames`, then watches the
/// locked patch with template matching until the dot pattern vanishes.
/// Searches the whole frame (refactor 09, 3.7).
class BallPresenceTrigger : public ITriggerDetector, public IDiagnosticProvider {
private:
    DotClusterFinder finder;
    DotClusterResult result;
    int          lockFrameCount;
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
    cv::Mat      gray;
    cv::Mat      matchResult;

    nlohmann::json latestDiag;

public:
    BallPresenceTrigger(
        DotClusterConfig dotConfig = DotClusterConfig(),
        double nominalBallRadiusPx = 23.3,
        int lockFrames = 30,
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
