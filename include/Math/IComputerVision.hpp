#pragma once
#include <opencv2/core.hpp>
#include <vector>

struct MarkerObservation {
    cv::Point2d position; // 2D sub-pixel position in the image
    double intensity;     // peak or average brightness of the glint
};

struct BallObservation {
    cv::Point2d centroid; // 2D sub-pixel centroid of the ball
    cv::Rect boundingBox; // Bounding box around the ball silhouette
    std::vector<MarkerObservation> markers; // Visible marker glints in this ball
};

class IComputerVision {
public:
    virtual ~IComputerVision() = default;
    virtual std::vector<BallObservation> detectBalls(const cv::Mat& frame) = 0;
};
