#pragma once
#include "Math/ITriggerDetector.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Diagnostics/IDiagnosticProvider.hpp"

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <vector>
#include <cstdint>

enum class StereoTriggerState {
    SEARCHING,
    ARMED,
    CONFIRMING,
    CAPTURED
};

struct BlobCandidate {
    cv::Point2d centroid;
    cv::Point2d rectCentroid;
    cv::Rect    boundingRect;
    double      area;
    double      radius;
    double      circularity;
};

class StereoBallTrackerTrigger : public ITriggerDetector, public IDiagnosticProvider {
private:
    StereoCalibration  calib_;
    StereoTriggerState state_;

    // Target optical parameters for 2 ft (~600 mm) setup
    cv::Rect           searchRoiLeft_;
    cv::Rect           searchRoiRight_;
    double             minBallRadiusPx_;
    double             maxBallRadiusPx_;
    double             minCircularity_;
    int                ballThreshold_;
    double             epipolarTolerancePx_;
    double             disparityMinPx_;
    double             disparityMaxPx_;
    int                armedWindowSize_;
    int                graceWindowMax_;
    double             impactVelocityThreshold_;      // in m/s (default 4.0 m/s ~ 9 mph)
    double             motionDisplacementThreshold_;  // in meters (default 0.04 m ~ 2 ball radii)

    // Tracking state across frames
    Eigen::Vector3d    lastKnown3DPos_;
    cv::Point2d        lastKnownLeft2D_;
    cv::Point2d        lastKnownRight2D_;
    cv::Point2d        lastKnownLeftRect2D_;
    cv::Point2d        lastKnownRightRect2D_;

    int                graceCounter_;
    int                confirmFrames_;
    std::vector<Eigen::Vector3d> confirmPositions3D_;

    // Zero-allocation scratchpad matrices
    cv::Mat            grayL_;
    cv::Mat            grayR_;
    cv::Mat            threshL_;
    cv::Mat            threshR_;
    cv::Mat            pt_temp_;
    cv::Mat            und_temp_;
    cv::Mat            pt2D_L_;
    cv::Mat            pt2D_R_;
    cv::Mat            undL_;
    cv::Mat            undR_;
    cv::Mat            pt4D_;

    nlohmann::json     latestDiag_;

    // Helper functions
    std::vector<BlobCandidate> extractCandidates(
        const cv::Mat& grayFrame,
        const cv::Rect& searchROI,
        const cv::Mat& K, const cv::Mat& D, const cv::Mat& R_rect, const cv::Mat& P_rect
    );

    cv::Point2d rectifyPoint(
        const cv::Point2d& pt,
        const cv::Mat& K, const cv::Mat& D, const cv::Mat& R_rect, const cv::Mat& P_rect
    );

    bool triangulateCentroid(
        const cv::Point2d& leftPt,
        const cv::Point2d& rightPt,
        Eigen::Vector3d& out3D
    );

    cv::Point2d project3DToLeft(const Eigen::Vector3d& pt3D);
    cv::Point2d project3DToRight(const Eigen::Vector3d& pt3D);

public:
    StereoBallTrackerTrigger(
        StereoCalibration calib = StereoCalibration(),
        cv::Rect searchRoiLeft = cv::Rect(300, 300, 400, 400),
        cv::Rect searchRoiRight = cv::Rect(300, 300, 400, 400),
        double minRadius = 30.0,
        double maxRadius = 50.0,
        double minCirc = 0.65,
        int thresh = 100,
        double epipolarTol = 3.0,
        int armedWinSize = 256,
        int graceMax = 4,
        double impactVelThresh = 4.0,
        double motionDispThresh = 0.04
    );

    bool checkTrigger(const cv::Mat& leftFrame, const cv::Mat& rightFrame) override;
    bool checkOpticalGate(const cv::Mat& currentFrame) override;
    void reset() override;

    StereoTriggerState getState() const { return state_; }
    Eigen::Vector3d getLastKnown3DPosition() const { return lastKnown3DPos_; }

    nlohmann::json getLatestDiagnostics() const override {
        return latestDiag_;
    }
};
