#pragma once
#include "Math/ISpatialSolver.hpp"
#include <opencv2/core.hpp>

struct StereoCalibration {
    cv::Mat K_L;
    cv::Mat D_L;
    cv::Mat K_R;
    cv::Mat D_R;
    cv::Mat R; // Rotation mapping Right to Left
    cv::Mat T; // Translation mapping Right to Left
    cv::Mat R_L, R_R, P_L, P_R, Q; // Rectification matrices
};

class StereoTriangulator : public ISpatialSolver {
private:
    StereoCalibration calib_;
    double            ballRadius_; // in meters (default 0.021335)
    
    // Scratchpads to avoid heap allocation in the hot path
    cv::Mat           pt2D_L_;
    cv::Mat           pt2D_R_;
    cv::Mat           undL_;
    cv::Mat           undR_;
    cv::Mat           pt4D_;
    cv::Mat           pt_temp_;
    cv::Mat           und_temp_;
public:
    StereoTriangulator();
    StereoTriangulator(const StereoCalibration& calib, double ballRadius = 0.021335);

    void setCalibration(const StereoCalibration& calib);

    std::vector<Ball3D> triangulateShot(
        const std::vector<BallObservation>& leftObs,
        const std::vector<BallObservation>& rightObs
    ) override;
};

