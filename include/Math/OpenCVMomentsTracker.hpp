#pragma once
#include "Math/IComputerVision.hpp"

class OpenCVMomentsTracker : public IComputerVision {
private:
    int    ballThreshold_;
    int    markerThreshold_;
    double minBallArea_;
    double maxBallArea_;
    double minBallCircularity_;

    // Scratchpad variables to avoid allocations in hot path
    cv::Mat gray_;
    cv::Mat ballMask_;
    cv::Mat localRegion_;
    cv::Mat localRegionBlurred_;
    cv::Mat thresh_;

    std::vector<MarkerObservation> extractMarkersInROI(
        const cv::Mat& gray, 
        const cv::Rect& roi, 
        int markerThreshold
    );
public:
    OpenCVMomentsTracker(
        int ballThresh = 60, 
        int markerThresh = 220, 
        double minBAr = 200, 
        double maxBAr = 10000, 
        double minBCirc = 0.6
    );

    std::vector<BallObservation> detectBalls(const cv::Mat& frame) override;
};

