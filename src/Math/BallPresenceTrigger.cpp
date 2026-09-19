#include "Math/BallPresenceTrigger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry.hpp>
#include <spdlog/spdlog.h>
#include <cmath>

BallPresenceTrigger::BallPresenceTrigger(
    DotClusterConfig dotConfig,
    double nominalBallRadiusPx,
    int lockFrames,
    float matchThreshold,
    double lossTimeout
) : finder(dotConfig, nominalBallRadiusPx),
    lockFrameCount(lockFrames),
    matchScoreThreshold(matchThreshold),
    lossTimeoutSec(lossTimeout),
    state(TriggerState::WAITING_FOR_BALL),
    stabilityCounter(0),
    lastCandidateCentroid(0, 0),
    hasEmptyStartTime(false),
    emitterMode(EmitterPowerMode::HIGH_STROBE_READY) {

    latestDiag = {
        {"state", "WAITING_FOR_BALL"},
        {"triggered", false},
        {"stabilityCounter", 0},
        {"lockFrameCount", lockFrameCount},
        {"matchScore", 1.0f},
        {"lockedBallBox", {0, 0, 0, 0}},
        {"emitterMode", "READY"},
        {"emptyDurationSec", 0.0}
    };
}

bool BallPresenceTrigger::checkTrigger(const cv::Mat& leftFrame, const cv::Mat& /*rightFrame*/) {
    return checkOpticalGate(leftFrame);
}

bool BallPresenceTrigger::checkOpticalGate(const cv::Mat& currentFrame) {
    if (currentFrame.empty()) {
        return false;
    }

    if (currentFrame.channels() == 3) {
        cv::cvtColor(currentFrame, gray, cv::COLOR_BGR2GRAY);
    } else if (currentFrame.channels() == 4) {
        cv::cvtColor(currentFrame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = currentFrame;
    }

    if (state == TriggerState::WAITING_FOR_BALL) {
        // Whole-frame dot-cluster search: no tee ROI (refactor 09, 3.7)
        finder.find(gray, result);

        cv::Point2d validCentroid(0, 0);
        cv::Rect validBoundBox(0, 0, 0, 0);
        int validCount = static_cast<int>(result.clusters.size());
        if (validCount > 0) {
            validCentroid = result.clusters[0].centroid;
            validBoundBox = result.clusters[0].boundingBox;
        }

        if (validCount == 1) {
            // Ball candidate detected -> restore High Strobe Ready
            hasEmptyStartTime = false;
            emitterMode = EmitterPowerMode::HIGH_STROBE_READY;

            double dist = cv::norm(validCentroid - lastCandidateCentroid);
            if (stabilityCounter > 0 && dist < 15.0) {
                stabilityCounter++;
            } else {
                stabilityCounter = 1;
            }
            lastCandidateCentroid = validCentroid;

            if (stabilityCounter >= lockFrameCount) {
                state = TriggerState::BALL_LOCKED;
                lockedBallBox = validBoundBox;

                // Extract and store 2D pixel template of locked ball in grayscale
                cv::Rect safeLockRoi = lockedBallBox & cv::Rect(0, 0, gray.cols, gray.rows);
                if (safeLockRoi.area() > 0) {
                    lockedBallTemplate = gray(safeLockRoi).clone();
                }

                spdlog::info("[BallPresenceTrigger] Ball locked at ({:.1f}, {:.1f}) after {} stable frames! Stored 2D pixel template ({}x{}).",
                             validCentroid.x, validCentroid.y, stabilityCounter, lockedBallTemplate.cols, lockedBallTemplate.rows);
            }
        } else {
            stabilityCounter = 0;

            // Track how long tee has been empty to enforce emitter protection
            auto now = std::chrono::steady_clock::now();
            if (!hasEmptyStartTime) {
                emptyStartTime = now;
                hasEmptyStartTime = true;
            } else {
                double emptySec = std::chrono::duration<double>(now - emptyStartTime).count();
                if (emptySec >= lossTimeoutSec) {
                    if (emitterMode != EmitterPowerMode::LOW_STANDBY) {
                        spdlog::info("[BallPresenceTrigger] Tee unoccupied for {:.1f}s >= {:.1f}s threshold. Entering LOW_STANDBY emitter protection.",
                                     emptySec, lossTimeoutSec);
                    }
                    emitterMode = EmitterPowerMode::LOW_STANDBY;
                }
            }
        }

        double emptySec = hasEmptyStartTime ? std::chrono::duration<double>(std::chrono::steady_clock::now() - emptyStartTime).count() : 0.0;
        latestDiag = {
            {"state", (state == TriggerState::BALL_LOCKED) ? "BALL_LOCKED" : "WAITING_FOR_BALL"},
            {"triggered", false},
            {"stabilityCounter", stabilityCounter},
            {"lockFrameCount", lockFrameCount},
            {"matchScore", 1.0f},
            {"candidateCentroid", {validCentroid.x, validCentroid.y}},
            {"candidateCount", validCount},
            {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}},
            {"emitterMode", (emitterMode == EmitterPowerMode::LOW_STANDBY) ? "STANDBY" : "READY"},
            {"emptyDurationSec", emptySec},
            {"dots", result.toJson()}
        };
        return false;

    } else if (state == TriggerState::BALL_LOCKED) {
        hasEmptyStartTime = false;
        emitterMode = EmitterPowerMode::HIGH_STROBE_READY;

        cv::Rect safeLockRoi = lockedBallBox & cv::Rect(0, 0, gray.cols, gray.rows);
        float matchScore = 0.0f;

        if (safeLockRoi.area() > 0 && !lockedBallTemplate.empty() && safeLockRoi.size() == lockedBallTemplate.size()) {
            cv::Mat currentGray = gray(safeLockRoi);

            // Run 2D Normalized Cross-Correlation Template Matching
            cv::matchTemplate(currentGray, lockedBallTemplate, matchResult, cv::TM_CCOEFF_NORMED);
            matchScore = matchResult.at<float>(0, 0);
        }

        // Check if match score dropped below threshold (ball structure pattern vanished from tee)
        bool departed = (matchScore < matchScoreThreshold);

        if (departed) {
            state = TriggerState::BALL_DEPARTED;
            emptyStartTime = std::chrono::steady_clock::now();
            hasEmptyStartTime = true;

            spdlog::info("[BallPresenceTrigger] Ball departed! 2D pixel match score {:.2f} < threshold {:.2f}. Triggering shot!",
                         matchScore, matchScoreThreshold);

            latestDiag = {
                {"state", "BALL_DEPARTED"},
                {"triggered", true},
                {"stabilityCounter", stabilityCounter},
                {"lockFrameCount", lockFrameCount},
                {"matchScore", matchScore},
                {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}},
                {"emitterMode", "READY"},
                {"emptyDurationSec", 0.0}
            };
            return true;
        } else {
            latestDiag = {
                {"state", "BALL_LOCKED"},
                {"triggered", false},
                {"stabilityCounter", stabilityCounter},
                {"lockFrameCount", lockFrameCount},
                {"matchScore", matchScore},
                {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}},
                {"emitterMode", "READY"},
                {"emptyDurationSec", 0.0}
            };
            return false;
        }

    } else if (state == TriggerState::BALL_DEPARTED) {
        auto now = std::chrono::steady_clock::now();
        if (!hasEmptyStartTime) {
            emptyStartTime = now;
            hasEmptyStartTime = true;
        } else {
            double emptySec = std::chrono::duration<double>(now - emptyStartTime).count();
            if (emptySec >= lossTimeoutSec) {
                emitterMode = EmitterPowerMode::LOW_STANDBY;
            }
        }

        double emptySec = hasEmptyStartTime ? std::chrono::duration<double>(std::chrono::steady_clock::now() - emptyStartTime).count() : 0.0;
        latestDiag = {
            {"state", "BALL_DEPARTED"},
            {"triggered", false},
            {"stabilityCounter", stabilityCounter},
            {"lockFrameCount", lockFrameCount},
            {"matchScore", 0.0f},
            {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}},
            {"emitterMode", (emitterMode == EmitterPowerMode::LOW_STANDBY) ? "STANDBY" : "READY"},
            {"emptyDurationSec", emptySec}
        };
        return false;
    }

    return false;
}

void BallPresenceTrigger::reset() {
    state = TriggerState::WAITING_FOR_BALL;
    stabilityCounter = 0;
    lastCandidateCentroid = cv::Point2d(0, 0);
    lockedBallBox = cv::Rect(0, 0, 0, 0);
    lockedBallTemplate.release();
    hasEmptyStartTime = false;
    emitterMode = EmitterPowerMode::HIGH_STROBE_READY;

    latestDiag = {
        {"state", "WAITING_FOR_BALL"},
        {"triggered", false},
        {"stabilityCounter", 0},
        {"lockFrameCount", lockFrameCount},
        {"matchScore", 1.0f},
        {"lockedBallBox", {0, 0, 0, 0}},
        {"emitterMode", "READY"},
        {"emptyDurationSec", 0.0}
    };
}
