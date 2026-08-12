#include "Math/StereoBallTrackerTrigger.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

StereoBallTrackerTrigger::StereoBallTrackerTrigger(
    StereoCalibration calib, cv::Rect searchRoiLeft, cv::Rect searchRoiRight,
    double minRadius, double maxRadius, double minCirc, int thresh,
    double epipolarTol, int armedWinSize, double max3DDist,
    int searchLockFrames, int graceMax, double impactVelThresh,
    double motionDispThresh, double minArea)
    : calib_(calib), state_(StereoTriggerState::SEARCHING),
      searchRoiLeft_(searchRoiLeft), searchRoiRight_(searchRoiRight),
      minBallRadiusPx_(minRadius), maxBallRadiusPx_(maxRadius),
      minBallArea_(minArea), minCircularity_(minCirc), ballThreshold_(thresh),
      epipolarTolerancePx_(epipolarTol), disparityMinPx_(10.0),
      disparityMaxPx_(400.0), armedWindowSize_(armedWinSize),
      max3DDistanceMeters_(max3DDist), minZDistanceMeters_(0.20),
      searchLockFrameCount_(searchLockFrames), searchStabilityCounter_(0),
      lastSearchCandidate3D_(0.0, 0.0, 0.0),
      searchStabilityToleranceMeters_(0.020), graceWindowMax_(graceMax),
      impactVelocityThreshold_(impactVelThresh),
      motionDisplacementThreshold_(motionDispThresh),
      lastKnown3DPos_(0.0, 0.0, 0.0), lastKnownLeft2D_(0.0, 0.0),
      lastKnownRight2D_(0.0, 0.0), lastKnownLeftRect2D_(0.0, 0.0),
      lastKnownRightRect2D_(0.0, 0.0), graceCounter_(0), confirmFrames_(0),
      searchingLogCounter_(0) {

  // Populate fallback default calibration if matrices are uninitialized
  if (calib_.K_L.empty()) {
    calib_.K_L = (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 640.0, 0.0, 1000.0,
                  400.0, 0.0, 0.0, 1.0);
    calib_.D_L = cv::Mat::zeros(1, 5, CV_64F);
    calib_.K_R = calib_.K_L.clone();
    calib_.D_R = calib_.D_L.clone();

    calib_.R = cv::Mat::eye(3, 3, CV_64F);
    calib_.T =
        (cv::Mat_<double>(3, 1) << -0.1, 0.0, 0.0); // 100mm baseline along X

    calib_.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib_.R_R = cv::Mat::eye(3, 3, CV_64F);

    calib_.P_L = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, 0.0, 0.0,
                  1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    calib_.P_R = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, -100.0, 0.0,
                  1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  }

  spdlog::info("[StereoTrigger] State initialized: SEARCHING for ball...");

  latestDiag_ = {{"state", "SEARCHING"},
                 {"triggered", false},
                 {"graceCounter", 0},
                 {"searchStabilityCounter", 0},
                 {"lastKnown3D", {0.0, 0.0, 0.0}}};
}

cv::Point2d StereoBallTrackerTrigger::rectifyPoint(const cv::Point2d &pt,
                                                   const cv::Mat &K,
                                                   const cv::Mat &D,
                                                   const cv::Mat &R_rect,
                                                   const cv::Mat &P_rect) {
  pt_temp_.create(1, 1, CV_64FC2);
  pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(pt.x, pt.y);
  cv::undistortPoints(pt_temp_, und_temp_, K, D, R_rect, P_rect);

  cv::Mat und_double;
  if (und_temp_.depth() != CV_64F) {
    und_temp_.convertTo(und_double, CV_64F);
  } else {
    und_double = und_temp_;
  }
  return cv::Point2d(und_double.at<double>(0, 0), und_double.at<double>(0, 1));
}

bool StereoBallTrackerTrigger::triangulateCentroid(const cv::Point2d &leftPt,
                                                   const cv::Point2d &rightPt,
                                                   Eigen::Vector3d &out3D) {
  cv::Point2d rectL =
      rectifyPoint(leftPt, calib_.K_L, calib_.D_L, calib_.R_L, calib_.P_L);
  cv::Point2d rectR =
      rectifyPoint(rightPt, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);

  pt2D_L_.create(1, 1, CV_64FC2);
  pt2D_L_.at<cv::Vec2d>(0, 0) = cv::Vec2d(rectL.x, rectL.y);
  pt2D_R_.create(1, 1, CV_64FC2);
  pt2D_R_.at<cv::Vec2d>(0, 0) = cv::Vec2d(rectR.x, rectR.y);

  cv::triangulatePoints(calib_.P_L, calib_.P_R, pt2D_L_, pt2D_R_, pt4D_);
  double w = pt4D_.at<double>(3, 0);
  if (std::abs(w) < 1e-6) {
    return false;
  }

  out3D =
      Eigen::Vector3d(pt4D_.at<double>(0, 0) / w, pt4D_.at<double>(1, 0) / w,
                      pt4D_.at<double>(2, 0) / w);
  return true;
}

cv::Point2d
StereoBallTrackerTrigger::project3DToLeft(const Eigen::Vector3d &pt3D) {
  cv::Mat ptHom = (cv::Mat_<double>(4, 1) << pt3D.x(), pt3D.y(), pt3D.z(), 1.0);
  cv::Mat proj = calib_.P_L * ptHom;
  double w = proj.at<double>(2, 0);
  if (std::abs(w) < 1e-6)
    return cv::Point2d(640, 400);
  return cv::Point2d(proj.at<double>(0, 0) / w, proj.at<double>(1, 0) / w);
}

cv::Point2d
StereoBallTrackerTrigger::project3DToRight(const Eigen::Vector3d &pt3D) {
  cv::Mat ptHom = (cv::Mat_<double>(4, 1) << pt3D.x(), pt3D.y(), pt3D.z(), 1.0);
  cv::Mat proj = calib_.P_R * ptHom;
  double w = proj.at<double>(2, 0);
  if (std::abs(w) < 1e-6)
    return cv::Point2d(640, 400);
  return cv::Point2d(proj.at<double>(0, 0) / w, proj.at<double>(1, 0) / w);
}

std::vector<BlobCandidate> StereoBallTrackerTrigger::extractCandidates(
    const cv::Mat &grayFrame, const cv::Rect &searchROI, const cv::Mat &K,
    const cv::Mat &D, const cv::Mat &R_rect, const cv::Mat &P_rect) {
  std::vector<BlobCandidate> candidates;
  if (grayFrame.empty())
    return candidates;

  cv::Rect safeRoi = searchROI & cv::Rect(0, 0, grayFrame.cols, grayFrame.rows);
  if (safeRoi.area() <= 0)
    return candidates;

  cv::Mat roiFrame = grayFrame(safeRoi);
  cv::Mat blurred;
  cv::GaussianBlur(roiFrame, blurred, cv::Size(3, 3), 0);

  // Binary threshold using ballThreshold_ (separates bright ball dome from
  // carpet background)
  cv::Mat threshRoi;
  cv::threshold(blurred, threshRoi, ballThreshold_, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(threshRoi, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  for (const auto &contour : contours) {
    double area = cv::contourArea(contour);
    if (area < minBallArea_)
      continue;

    cv::Point2f center;
    float radius = 0.0f;
    cv::minEnclosingCircle(contour, center, radius);

    if (radius < minBallRadiusPx_ || radius > maxBallRadiusPx_) {
      continue;
    }

    double perimeter = cv::arcLength(contour, true);
    double circularity = 0.0;
    if (perimeter > 0.0) {
      circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
    }

    if (circularity >= minCircularity_) {
      cv::Moments m = cv::moments(contour);
      if (m.m00 > 0.0) {
        cv::Point2d localCentroid(m.m10 / m.m00, m.m01 / m.m00);
        cv::Point2d globalCentroid(safeRoi.x + localCentroid.x,
                                   safeRoi.y + localCentroid.y);
        cv::Rect localBox = cv::boundingRect(contour);
        cv::Rect globalBox(safeRoi.x + localBox.x, safeRoi.y + localBox.y,
                           localBox.width, localBox.height);

        cv::Point2d rectCen =
            rectifyPoint(globalCentroid, K, D, R_rect, P_rect);

        BlobCandidate cand;
        cand.centroid = globalCentroid;
        cand.rectCentroid = rectCen;
        cand.boundingRect = globalBox;
        cand.area = area;
        cand.radius = static_cast<double>(radius);
        cand.circularity = circularity;
        candidates.push_back(cand);
      }
    }
  }

  return candidates;
}

bool StereoBallTrackerTrigger::checkTrigger(const cv::Mat &leftFrame,
                                            const cv::Mat &rightFrame) {
  if (leftFrame.empty() || rightFrame.empty()) {
    return false;
  }

  // Ensure grayscale
  if (leftFrame.channels() == 3) {
    cv::cvtColor(leftFrame, grayL_, cv::COLOR_BGR2GRAY);
  } else if (leftFrame.channels() == 4) {
    cv::cvtColor(leftFrame, grayL_, cv::COLOR_BGRA2GRAY);
  } else {
    grayL_ = leftFrame;
  }

  if (rightFrame.channels() == 3) {
    cv::cvtColor(rightFrame, grayR_, cv::COLOR_BGR2GRAY);
  } else if (rightFrame.channels() == 4) {
    cv::cvtColor(rightFrame, grayR_, cv::COLOR_BGRA2GRAY);
  } else {
    grayR_ = rightFrame;
  }

  if (state_ == StereoTriggerState::SEARCHING) {
    auto leftCandidates = extractCandidates(grayL_, searchRoiLeft_, calib_.K_L,
                                            calib_.D_L, calib_.R_L, calib_.P_L);
    auto rightCandidates =
        extractCandidates(grayR_, searchRoiRight_, calib_.K_R, calib_.D_R,
                          calib_.R_R, calib_.P_R);

    const BlobCandidate *bestL = nullptr;
    const BlobCandidate *bestR = nullptr;
    Eigen::Vector3d best3D(0.0, 0.0, 0.0);
    double minPairScore = 1e9;
    double bestPairEpipolarErr = 999.0;
    double bestPairDist3D = 0.0;

    for (const auto &candL : leftCandidates) {
      for (const auto &candR : rightCandidates) {
        double epiErr = std::abs(candL.rectCentroid.y - candR.rectCentroid.y);
        double disparity = candL.rectCentroid.x - candR.rectCentroid.x;

        if (epiErr <= epipolarTolerancePx_ && disparity >= disparityMinPx_ &&
            disparity <= disparityMaxPx_) {
          Eigen::Vector3d cand3D;
          if (triangulateCentroid(candL.centroid, candR.centroid, cand3D)) {
            double dist3D = cand3D.norm();

            // Strict 3D Distance Constraint: Ball must be within 3ft (0.9144m)
            // of cameras and in front of cameras (Z >= minZDistanceMeters_)
            if (dist3D <= max3DDistanceMeters_ &&
                cand3D.z() >= minZDistanceMeters_) {
              // Composite Pair Quality Score: Epipolar alignment + Radius
              // symmetry + Circularity
              double maxRad = std::max(candL.radius, candR.radius);
              double radDiffRatio =
                  (maxRad > 0.0)
                      ? (std::abs(candL.radius - candR.radius) / maxRad)
                      : 0.0;
              double circPenalty =
                  1.0 - (candL.circularity * candR.circularity);

              double score =
                  epiErr + (10.0 * radDiffRatio) + (50.0 * circPenalty);
              if (score < minPairScore) {
                minPairScore = score;
                bestL = &candL;
                bestR = &candR;
                best3D = cand3D;
                bestPairEpipolarErr = epiErr;
                bestPairDist3D = dist3D;
              }
            }
          }
        }
      }
    }

    if (bestL != nullptr && bestR != nullptr) {
      // Check 3D positional stability across consecutive frames before locking
      double posShiftMeters = (best3D - lastSearchCandidate3D_).norm();
      if (searchStabilityCounter_ > 0 &&
          posShiftMeters <= searchStabilityToleranceMeters_) {
        searchStabilityCounter_++;
      } else {
        searchStabilityCounter_ = 1;
      }
      lastSearchCandidate3D_ = best3D;

      if (searchStabilityCounter_ >= searchLockFrameCount_) {
        lastKnown3DPos_ = best3D;
        lastKnownLeft2D_ = bestL->centroid;
        lastKnownRight2D_ = bestR->centroid;
        lastKnownLeftRect2D_ = bestL->rectCentroid;
        lastKnownRightRect2D_ = bestR->rectCentroid;
        graceCounter_ = 0;
        searchingLogCounter_ = 0;

        state_ = StereoTriggerState::ARMED;
        double distFeet = bestPairDist3D * 3.28084;
        double distMm = bestPairDist3D * 1000.0;
        spdlog::info(
            "[StereoTrigger] State change: SEARCHING -> ARMED after {} stable "
            "frames! "
            "(Ball locked at 3D pos: X={:.2f}m, Y={:.2f}m, Z={:.2f}m | "
            "Distance from cameras: {:.2f}m / {:.2f} ft / {:.0f} mm)",
            searchStabilityCounter_, best3D.x(), best3D.y(), best3D.z(),
            bestPairDist3D, distFeet, distMm);
      }
    } else {
      searchStabilityCounter_ = 0;
      searchingLogCounter_++;
      if (searchingLogCounter_ % 60 == 1) {
        spdlog::info(
            "[StereoTrigger] Searching... Candidates found: Left={}, Right={}. "
            "(Threshold: {}, Radius: {:.0f}-{:.0f}px, Max 3D Distance: {:.2f}m "
            "/ 3.0ft)",
            leftCandidates.size(), rightCandidates.size(), ballThreshold_,
            minBallRadiusPx_, maxBallRadiusPx_, max3DDistanceMeters_);
      }
    }

    latestDiag_ = {
        {"state", "SEARCHING"},
        {"triggered", false},
        {"leftCandidates", leftCandidates.size()},
        {"rightCandidates", rightCandidates.size()},
        {"searchStabilityCounter", searchStabilityCounter_},
        {"minPairScore", (minPairScore < 1e8) ? minPairScore : 999.0},
        {"minEpipolarErr",
         (bestPairEpipolarErr < 900.0) ? bestPairEpipolarErr : 999.0},
        {"cameraDistanceMeters", bestPairDist3D},
        {"lastKnown3D",
         {lastKnown3DPos_.x(), lastKnown3DPos_.y(), lastKnown3DPos_.z()}}};
    return false;

  } else if (state_ == StereoTriggerState::ARMED) {
    int halfWin = armedWindowSize_ / 2;
    cv::Rect winL(static_cast<int>(lastKnownLeft2D_.x) - halfWin,
                  static_cast<int>(lastKnownLeft2D_.y) - halfWin,
                  armedWindowSize_, armedWindowSize_);
    cv::Rect winR(static_cast<int>(lastKnownRight2D_.x) - halfWin,
                  static_cast<int>(lastKnownRight2D_.y) - halfWin,
                  armedWindowSize_, armedWindowSize_);

    auto leftCandidates = extractCandidates(grayL_, winL, calib_.K_L,
                                            calib_.D_L, calib_.R_L, calib_.P_L);
    auto rightCandidates = extractCandidates(
        grayR_, winR, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);

    bool leftFound = !leftCandidates.empty();
    bool rightFound = !rightCandidates.empty();

    if (leftFound && rightFound) {
      // Find best matching pair
      const BlobCandidate *bestL = nullptr;
      const BlobCandidate *bestR = nullptr;
      double minErr = 1e9;

      for (const auto &candL : leftCandidates) {
        for (const auto &candR : rightCandidates) {
          double epiErr = std::abs(candL.rectCentroid.y - candR.rectCentroid.y);
          if (epiErr < minErr) {
            minErr = epiErr;
            bestL = &candL;
            bestR = &candR;
          }
        }
      }

      if (bestL && bestR) {
        Eigen::Vector3d curr3D;
        if (triangulateCentroid(bestL->centroid, bestR->centroid, curr3D)) {
          double disp3D = (curr3D - lastKnown3DPos_).norm();

          if (disp3D > motionDisplacementThreshold_) {
            state_ = StereoTriggerState::CONFIRMING;
            confirmFrames_ = 1;
            confirmPositions3D_ = {lastKnown3DPos_, curr3D};
            lastKnown3DPos_ = curr3D;
            lastKnownLeft2D_ = bestL->centroid;
            lastKnownRight2D_ = bestR->centroid;

            spdlog::info(
                "[StereoTrigger] State change: ARMED -> CONFIRMING (Motion "
                "detected: {:.1f} mm displacement > {:.1f} mm threshold)",
                disp3D * 1000.0, motionDisplacementThreshold_ * 1000.0);
          } else {
            // Stable tracking update
            lastKnown3DPos_ = curr3D;
            lastKnownLeft2D_ = bestL->centroid;
            lastKnownRight2D_ = bestR->centroid;
            graceCounter_ = 0;
          }
        }
      }
    } else if (leftFound != rightFound) {
      // SINGLE-CAMERA OCCLUSION IMMUNITY RULE:
      // Hand/club passing behind ball occludes one camera angle.
      // Hold last known 3D position! Do not trigger, do not drop back to
      // SEARCHING.
      spdlog::debug("[StereoTrigger] Single-camera occlusion immunity active "
                    "(Left: {}, Right: {}). Holding ARMED state.",
                    leftFound, rightFound);
    } else {
      // Both lost simultaneously
      graceCounter_++;
      if (graceCounter_ > graceWindowMax_) {
        state_ = StereoTriggerState::SEARCHING;
        graceCounter_ = 0;
        spdlog::info("[StereoTrigger] State change: ARMED -> SEARCHING (Ball "
                     "removed from tee / lost for {} frames)",
                     graceWindowMax_);
      }
    }

    latestDiag_ = {
        {"state",
         (state_ == StereoTriggerState::ARMED) ? "ARMED" : "CONFIRMING"},
        {"triggered", false},
        {"leftFound", leftFound},
        {"rightFound", rightFound},
        {"graceCounter", graceCounter_},
        {"lastKnown3D",
         {lastKnown3DPos_.x(), lastKnown3DPos_.y(), lastKnown3DPos_.z()}}};
    return false;

  } else if (state_ == StereoTriggerState::CONFIRMING) {
    int halfWin = armedWindowSize_ / 2;
    cv::Rect winL(static_cast<int>(lastKnownLeft2D_.x) - halfWin,
                  static_cast<int>(lastKnownLeft2D_.y) - halfWin,
                  armedWindowSize_, armedWindowSize_);
    cv::Rect winR(static_cast<int>(lastKnownRight2D_.x) - halfWin,
                  static_cast<int>(lastKnownRight2D_.y) - halfWin,
                  armedWindowSize_, armedWindowSize_);

    auto leftCandidates = extractCandidates(grayL_, winL, calib_.K_L,
                                            calib_.D_L, calib_.R_L, calib_.P_L);
    auto rightCandidates = extractCandidates(
        grayR_, winR, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);

    bool bothFound = (!leftCandidates.empty() && !rightCandidates.empty());

    if (bothFound) {
      Eigen::Vector3d curr3D;
      if (triangulateCentroid(leftCandidates[0].centroid,
                              rightCandidates[0].centroid, curr3D)) {
        confirmPositions3D_.push_back(curr3D);
        confirmFrames_++;

        // Estimate velocity over known inter-frame strobe interval dt = 0.001s
        // (1 ms)
        double dt = 0.001;
        Eigen::Vector3d velocity = (curr3D - lastKnown3DPos_) / dt;
        double speed = velocity.norm();

        lastKnown3DPos_ = curr3D;
        lastKnownLeft2D_ = leftCandidates[0].centroid;
        lastKnownRight2D_ = rightCandidates[0].centroid;

        if (speed >= impactVelocityThreshold_) {
          state_ = StereoTriggerState::CAPTURED;
          spdlog::info(
              "[StereoTrigger] State change: CONFIRMING -> CAPTURED (Shot hit "
              "confirmed! Velocity: {:.1f} mph / {:.2f} m/s)",
              speed * 2.23694, speed);

          latestDiag_ = {{"state", "CAPTURED"},
                         {"triggered", true},
                         {"impactVelocity", speed},
                         {"confirmFrames", confirmFrames_},
                         {"lastKnown3D",
                          {lastKnown3DPos_.x(), lastKnown3DPos_.y(),
                           lastKnown3DPos_.z()}}};
          return true;
        } else if (confirmFrames_ >= 3) {
          // Velocity failed threshold check -> Nudge/vibration rejection
          state_ = StereoTriggerState::ARMED;
          confirmFrames_ = 0;
          confirmPositions3D_.clear();
          spdlog::info("[StereoTrigger] State change: CONFIRMING -> ARMED "
                       "(Nudge/vibration rejected: speed {:.1f} mph < "
                       "threshold {:.1f} mph)",
                       speed * 2.23694, impactVelocityThreshold_ * 2.23694);
          return false;
        }
      }
    } else {
      // Ball exited camera view at high velocity (valid launch!)
      if (!confirmPositions3D_.empty()) {
        state_ = StereoTriggerState::CAPTURED;
        spdlog::info("[StereoTrigger] State change: CONFIRMING -> CAPTURED "
                     "(Ball departed view at launch velocity!)");

        latestDiag_ = {
            {"state", "CAPTURED"},
            {"triggered", true},
            {"confirmFrames", confirmFrames_},
            {"lastKnown3D",
             {lastKnown3DPos_.x(), lastKnown3DPos_.y(), lastKnown3DPos_.z()}}};
        return true;
      }
    }

    latestDiag_ = {
        {"state", "CONFIRMING"},
        {"triggered", false},
        {"confirmFrames", confirmFrames_},
        {"lastKnown3D",
         {lastKnown3DPos_.x(), lastKnown3DPos_.y(), lastKnown3DPos_.z()}}};
    return false;

  } else if (state_ == StereoTriggerState::CAPTURED) {
    latestDiag_ = {
        {"state", "CAPTURED"},
        {"triggered", true},
        {"lastKnown3D",
         {lastKnown3DPos_.x(), lastKnown3DPos_.y(), lastKnown3DPos_.z()}}};
    return true;
  }

  return false;
}

bool StereoBallTrackerTrigger::checkOpticalGate(const cv::Mat &currentFrame) {
  return checkTrigger(currentFrame, currentFrame);
}

void StereoBallTrackerTrigger::reset() {
  state_ = StereoTriggerState::SEARCHING;
  lastKnown3DPos_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  lastKnownLeft2D_ = cv::Point2d(0.0, 0.0);
  lastKnownRight2D_ = cv::Point2d(0.0, 0.0);
  lastKnownLeftRect2D_ = cv::Point2d(0.0, 0.0);
  lastKnownRightRect2D_ = cv::Point2d(0.0, 0.0);
  graceCounter_ = 0;
  confirmFrames_ = 0;
  confirmPositions3D_.clear();

  searchStabilityCounter_ = 0;
  lastSearchCandidate3D_ = Eigen::Vector3d(0.0, 0.0, 0.0);

  spdlog::info("[StereoTrigger] State reset: SEARCHING for ball...");

  latestDiag_ = {{"state", "SEARCHING"},
                 {"triggered", false},
                 {"graceCounter", 0},
                 {"searchStabilityCounter", 0},
                 {"lastKnown3D", {0.0, 0.0, 0.0}}};
}
