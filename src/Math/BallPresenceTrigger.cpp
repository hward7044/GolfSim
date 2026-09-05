#include "Math/BallPresenceTrigger.hpp"
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/geometry.hpp>)
#include <opencv2/geometry.hpp>
#endif
#include <spdlog/spdlog.h>
#include <cmath>

BallPresenceTrigger::BallPresenceTrigger(
    cv::Rect roi,
    int lockFrames,
    int thresh,
    double minArea,
    double maxArea,
    double minCirc,
    float matchThreshold
) : teeROI(roi),
    lockFrameCount(lockFrames),
    ballThreshold(thresh),
    minBallArea(minArea),
    maxBallArea(maxArea),
    minCircularity(minCirc),
    matchScoreThreshold(matchThreshold),
    state(TriggerState::WAITING_FOR_BALL),
    stabilityCounter(0),
    lastCandidateCentroid(0, 0) {
    
    latestDiag = {
        {"state", "WAITING_FOR_BALL"},
        {"triggered", false},
        {"stabilityCounter", 0},
        {"lockFrameCount", lockFrameCount},
        {"matchScore", 1.0f},
        {"teeROI", {teeROI.x, teeROI.y, teeROI.width, teeROI.height}},
        {"lockedBallBox", {0, 0, 0, 0}}
    };
}

bool BallPresenceTrigger::checkOpticalGate(const cv::Mat& currentFrame) {
    if (currentFrame.empty()) {
        return false;
    }

    cv::Rect safeTeeRoi = teeROI & cv::Rect(0, 0, currentFrame.cols, currentFrame.rows);
    if (safeTeeRoi.area() <= 0) {
        return false;
    }

    cv::Mat roiFrame = currentFrame(safeTeeRoi);
    if (roiFrame.channels() == 3) {
        cv::cvtColor(roiFrame, grayRoi, cv::COLOR_BGR2GRAY);
    } else if (roiFrame.channels() == 4) {
        cv::cvtColor(roiFrame, grayRoi, cv::COLOR_BGRA2GRAY);
    } else {
        grayRoi = roiFrame;
    }

    if (state == TriggerState::WAITING_FOR_BALL) {
        cv::threshold(grayRoi, threshRoi, ballThreshold, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(threshRoi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point2d validCentroid(0, 0);
        cv::Rect validBoundBox(0, 0, 0, 0);
        int validCount = 0;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < minBallArea || area > maxBallArea) {
                continue;
            }

            double perimeter = cv::arcLength(contour, true);
            double circularity = 0.0;
            if (perimeter > 0.0) {
                circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
            }

            if (circularity >= minCircularity) {
                cv::Moments m = cv::moments(contour);
                if (m.m00 > 0.0) {
                    cv::Point2d localCentroid(m.m10 / m.m00, m.m01 / m.m00);
                    cv::Point2d globalCentroid(safeTeeRoi.x + localCentroid.x, safeTeeRoi.y + localCentroid.y);
                    cv::Rect localBox = cv::boundingRect(contour);
                    cv::Rect globalBox(safeTeeRoi.x + localBox.x, safeTeeRoi.y + localBox.y, localBox.width, localBox.height);

                    validCentroid = globalCentroid;
                    validBoundBox = globalBox;
                    validCount++;
                }
            }
        }

        if (validCount == 1) {
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
                cv::Rect safeLockRoi = lockedBallBox & cv::Rect(0, 0, currentFrame.cols, currentFrame.rows);
                if (safeLockRoi.area() > 0) {
                    cv::Mat lockMat = currentFrame(safeLockRoi);
                    if (lockMat.channels() == 3) {
                        cv::cvtColor(lockMat, lockedBallTemplate, cv::COLOR_BGR2GRAY);
                    } else if (lockMat.channels() == 4) {
                        cv::cvtColor(lockMat, lockedBallTemplate, cv::COLOR_BGRA2GRAY);
                    } else {
                        lockedBallTemplate = lockMat.clone();
                    }
                }

                spdlog::info("[BallPresenceTrigger] Ball locked at ({:.1f}, {:.1f}) after {} stable frames! Stored 2D pixel template ({}x{}).",
                             validCentroid.x, validCentroid.y, stabilityCounter, lockedBallTemplate.cols, lockedBallTemplate.rows);
            }
        } else {
            stabilityCounter = 0;
        }

        latestDiag = {
            {"state", (state == TriggerState::BALL_LOCKED) ? "BALL_LOCKED" : "WAITING_FOR_BALL"},
            {"triggered", false},
            {"stabilityCounter", stabilityCounter},
            {"lockFrameCount", lockFrameCount},
            {"matchScore", 1.0f},
            {"teeROI", {safeTeeRoi.x, safeTeeRoi.y, safeTeeRoi.width, safeTeeRoi.height}},
            {"candidateCentroid", {validCentroid.x, validCentroid.y}},
            {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}}
        };
        return false;

    } else if (state == TriggerState::BALL_LOCKED) {
        cv::Rect safeLockRoi = lockedBallBox & cv::Rect(0, 0, currentFrame.cols, currentFrame.rows);
        float matchScore = 0.0f;

        if (safeLockRoi.area() > 0 && !lockedBallTemplate.empty() && safeLockRoi.size() == lockedBallTemplate.size()) {
            cv::Mat currentPatch = currentFrame(safeLockRoi);
            cv::Mat currentGray;
            if (currentPatch.channels() == 3) {
                cv::cvtColor(currentPatch, currentGray, cv::COLOR_BGR2GRAY);
            } else if (currentPatch.channels() == 4) {
                cv::cvtColor(currentPatch, currentGray, cv::COLOR_BGRA2GRAY);
            } else {
                currentGray = currentPatch;
            }

            // Run 2D Normalized Cross-Correlation Template Matching
            cv::matchTemplate(currentGray, lockedBallTemplate, matchResult, cv::TM_CCOEFF_NORMED);
            matchScore = matchResult.at<float>(0, 0);
        }

        // Check if match score dropped below threshold (ball structure pattern vanished from tee)
        bool departed = (matchScore < matchScoreThreshold);

        if (departed) {
            state = TriggerState::BALL_DEPARTED;
            spdlog::info("[BallPresenceTrigger] Ball departed! 2D pixel match score {:.2f} < threshold {:.2f}. Triggering shot!",
                         matchScore, matchScoreThreshold);

            latestDiag = {
                {"state", "BALL_DEPARTED"},
                {"triggered", true},
                {"stabilityCounter", stabilityCounter},
                {"lockFrameCount", lockFrameCount},
                {"matchScore", matchScore},
                {"teeROI", {safeTeeRoi.x, safeTeeRoi.y, safeTeeRoi.width, safeTeeRoi.height}},
                {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}}
            };
            return true;
        } else {
            latestDiag = {
                {"state", "BALL_LOCKED"},
                {"triggered", false},
                {"stabilityCounter", stabilityCounter},
                {"lockFrameCount", lockFrameCount},
                {"matchScore", matchScore},
                {"teeROI", {safeTeeRoi.x, safeTeeRoi.y, safeTeeRoi.width, safeTeeRoi.height}},
                {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}}
            };
            return false;
        }

    } else if (state == TriggerState::BALL_DEPARTED) {
        latestDiag = {
            {"state", "BALL_DEPARTED"},
            {"triggered", false},
            {"stabilityCounter", stabilityCounter},
            {"lockFrameCount", lockFrameCount},
            {"matchScore", 0.0f},
            {"teeROI", {safeTeeRoi.x, safeTeeRoi.y, safeTeeRoi.width, safeTeeRoi.height}},
            {"lockedBallBox", {lockedBallBox.x, lockedBallBox.y, lockedBallBox.width, lockedBallBox.height}}
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

    latestDiag = {
        {"state", "WAITING_FOR_BALL"},
        {"triggered", false},
        {"stabilityCounter", 0},
        {"lockFrameCount", lockFrameCount},
        {"matchScore", 1.0f},
        {"teeROI", {teeROI.x, teeROI.y, teeROI.width, teeROI.height}},
        {"lockedBallBox", {0, 0, 0, 0}}
    };
}
