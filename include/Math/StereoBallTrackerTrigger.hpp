#pragma once
#include "Diagnostics/IDiagnosticProvider.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/StereoTriangulator.hpp"

#include <Eigen/Core>
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
};

class StereoBallTrackerTrigger : public ITriggerDetector,
                                 public IDiagnosticProvider {
private:
  StereoCalibration calib_;
  StereoTriggerState state_;

  // Target optical parameters for 2 ft (~600 mm) setup
  cv::Rect searchRoiLeft_;
  cv::Rect searchRoiRight_;
  double minBallRadiusPx_;
  double maxBallRadiusPx_;
  double minBallArea_;
  double minCircularity_;
  int ballThreshold_;
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
  cv::Mat threshL_;
  cv::Mat threshR_;
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
  extractCandidates(const cv::Mat &grayFrame, const cv::Rect &searchROI,
                    const cv::Mat &K, const cv::Mat &D, const cv::Mat &R_rect,
                    const cv::Mat &P_rect);

  cv::Point2d rectifyPoint(const cv::Point2d &pt, const cv::Mat &K,
                           const cv::Mat &D, const cv::Mat &R_rect,
                           const cv::Mat &P_rect);

  bool triangulateCentroid(const cv::Point2d &leftPt,
                           const cv::Point2d &rightPt, Eigen::Vector3d &out3D);

  cv::Point2d project3DToLeft(const Eigen::Vector3d &pt3D);
  cv::Point2d project3DToRight(const Eigen::Vector3d &pt3D);

  int searchingLogCounter_;

public:
  StereoBallTrackerTrigger(
      StereoCalibration calib = StereoCalibration(),
      cv::Rect searchRoiLeft = cv::Rect(350, 440, 600, 310),
      cv::Rect searchRoiRight = cv::Rect(350, 440, 600, 310),
      double minRadius = 15.0, double maxRadius = 50.0, double minCirc = 0.45,
      int thresh = 120, double epipolarTol = 65.0, int armedWinSize = 256,
      double max3DDist = 0.9144, int searchLockFrames = 5, int graceMax = 4,
      double impactVelThresh = 4.0, double motionDispThresh = 0.04,
      double minArea = 150.0);

  bool checkTrigger(const cv::Mat &leftFrame,
                    const cv::Mat &rightFrame) override;
  bool checkOpticalGate(const cv::Mat &currentFrame) override;
  void reset() override;

  StereoTriggerState getState() const { return state_; }
  Eigen::Vector3d getLastKnown3DPosition() const { return lastKnown3DPos_; }

  nlohmann::json getLatestDiagnostics() const override { return latestDiag_; }
};
