#pragma once
#include "Math/ISpatialSolver.hpp"
#include <opencv2/core.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/stereo.hpp>

struct StereoCalibration {
    cv::Mat K_L;
    cv::Mat D_L;
    cv::Mat K_R;
    cv::Mat D_R;
    cv::Mat R; // Rotation mapping Left to Right (OpenCV convention: X_R = R * X_L + T)
    cv::Mat T; // Translation mapping Left to Right
    cv::Mat R_L, R_R, P_L, P_R, Q; // Rectification matrices
};

class StereoTriangulator : public ISpatialSolver {
private:
    StereoCalibration calib_;
    double            ballRadius_; // in meters (default 0.021335)
    double            flightDirectionX_ = 1.0; // +1.0 for left-to-right, -1.0 for right-to-left

    // Precomputed extrinsics for Right camera in world (Left camera) coordinates
    Eigen::Matrix3d   R_inv_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d   rightOpticalCenter_ = Eigen::Vector3d::Zero();
    
    // Scratchpads to avoid heap allocation in the hot path
    cv::Mat           pt2D_L_;
    cv::Mat           pt2D_R_;
    cv::Mat           undL_;
    cv::Mat           undR_;
    cv::Mat           pt4D_;
    cv::Mat           pt_temp_;
    cv::Mat           und_temp_;

    void updateExtrinsics();
public:
    StereoTriangulator();
    StereoTriangulator(const StereoCalibration& calib, double ballRadius = 0.021335);

    void setCalibration(const StereoCalibration& calib);
    void setFlightDirectionX(double dir) noexcept { flightDirectionX_ = (dir >= 0.0 ? 1.0 : -1.0); }
    double getFlightDirectionX() const noexcept { return flightDirectionX_; }

    std::vector<Ball3D> triangulateShot(
        const std::vector<BallObservation>& leftObs,
        const std::vector<BallObservation>& rightObs
    ) override;
};

