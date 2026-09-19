#pragma once
#include "Diagnostics/IDiagnosticProvider.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/EmitterPowerMode.hpp"
#include "Math/DotClusterFinder.hpp"

#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <vector>

enum class StereoTriggerState { SEARCHING, ARMED, CONFIRMING, CAPTURED };

struct BlobCandidate {
  cv::Point2d centroid;
  cv::Point2d rectCentroid;
  cv::Rect boundingRect;
  double area;
  double radius;
  double circularity;
  int dotCount = 0;
};

/// Stereo ball-presence trigger over the dot-cluster detector. Searches the
/// whole frame — with continuous 300 Hz strobing there is no time pressure
/// before the shot, so the ball may sit anywhere in view (refactor 09, 3.7).
class StereoBallTrackerTrigger : public ITriggerDetector,
                                 public IDiagnosticProvider {
private:
  StereoCalibration calib_;
  StereoTriggerState state_;

  // Dots-only detection, shared configuration with the vision stage
  DotClusterFinder finder_;
  DotClusterResult resultL_;
  DotClusterResult resultR_;
  double nominalBallRadiusPx_;

  // Stereo pairing gates (AppConfig.stereo)
  double epipolarTolerancePx_;
  double disparityMinPx_;
  double disparityMaxPx_;
  int armedWindowSize_;
  int graceWindowMax_;
  double impactVelocityThreshold_; // in m/s (default 4.0 m/s ~ 9 mph)
  double
      motionDisplacementThreshold_; // in meters (default 0.04 m ~ 2 ball radii)

  // 3D Triangulation & Search Locking Constraints
  double
      max3DDistanceMeters_; // Max allowed distance from cameras (3ft = 0.9144m)
  double minZDistanceMeters_;  // Min forward distance Z from cameras (default
                               // 0.20m)
  int searchLockFrameCount_;   // Required consecutive stable frames (default 5)
  int searchStabilityCounter_; // Stability counter during SEARCHING state
  Eigen::Vector3d
      lastSearchCandidate3D_; // Candidate position in previous search frame
  double searchStabilityToleranceMeters_; // Maximum allowed jitter between
                                          // search frames (0.020m)

  // Tracking state across frames
  Eigen::Vector3d lastKnown3DPos_;
  cv::Point2d lastKnownLeft2D_;
  cv::Point2d lastKnownRight2D_;
  cv::Point2d lastKnownLeftRect2D_;
  cv::Point2d lastKnownRightRect2D_;

  int graceCounter_;
  int confirmFrames_;
  std::vector<Eigen::Vector3d> confirmPositions3D_;

  // Zero-allocation scratchpad matrices
  cv::Mat grayL_;
  cv::Mat grayR_;
  cv::Mat pt_temp_;
  cv::Mat und_temp_;
  cv::Mat pt2D_L_;
  cv::Mat pt2D_R_;
  cv::Mat undL_;
  cv::Mat undR_;
  cv::Mat pt4D_;

  nlohmann::json latestDiag_;

  // Helper functions
  std::vector<BlobCandidate>
  extractCandidates(const cv::Mat &grayFrame, DotClusterResult &scratch,
                    const cv::Mat &K, const cv::Mat &D, const cv::Mat &R_rect,
                    const cv::Mat &P_rect);

  /// Candidates whose centroid lies inside the armed tracking window around
  /// `center` (ARMED / CONFIRMING follow the locked ball, not the whole frame).
  static std::vector<BlobCandidate>
  withinWindow(const std::vector<BlobCandidate> &candidates,
               const cv::Point2d &center, int windowSize);

  cv::Point2d rectifyPoint(const cv::Point2d &pt, const cv::Mat &K,
                           const cv::Mat &D, const cv::Mat &R_rect,
                           const cv::Mat &P_rect);

  bool triangulateCentroid(const cv::Point2d &leftPt,
                           const cv::Point2d &rightPt, Eigen::Vector3d &out3D);

  cv::Point2d project3DToLeft(const Eigen::Vector3d &pt3D);
  cv::Point2d project3DToRight(const Eigen::Vector3d &pt3D);

  int searchingLogCounter_;

  // Standby emitter protection tracking
  EmitterPowerMode emitterMode_ = EmitterPowerMode::HIGH_STROBE_READY;
  double ballLossTimeoutSec_ = 5.0;
  std::chrono::steady_clock::time_point emptyStartTime_;
  bool hasEmptyStartTime_ = false;

public:
  StereoBallTrackerTrigger(StereoCalibration calib = StereoCalibration(),
                           DotClusterConfig dotConfig = DotClusterConfig(),
                           double nominalBallRadiusPx = 23.3,
                           double epipolarTol = 150.0,
                           double disparityMin = 5.0, double disparityMax = 600.0,
                           int armedWinSize = 256, double max3DDist = 0.9144,
                           int searchLockFrames = 5, int graceMax = 4,
                           double impactVelThresh = 4.0,
                           double motionDispThresh = 0.04);

  bool checkTrigger(const cv::Mat &leftFrame,
                    const cv::Mat &rightFrame) override;
  void reset() override;

  StereoTriggerState getState() const { return state_; }
  Eigen::Vector3d getLastKnown3DPosition() const { return lastKnown3DPos_; }

  EmitterPowerMode getEmitterMode() const noexcept { return emitterMode_; }
  bool isStandbyRequested() const noexcept {
    return emitterMode_ == EmitterPowerMode::LOW_STANDBY;
  }
  void setLossTimeoutSec(double sec) noexcept { ballLossTimeoutSec_ = sec; }

  const DotClusterConfig &dotConfig() const noexcept { return finder_.config(); }

  nlohmann::json getLatestDiagnostics() const override { return latestDiag_; }
};
