#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct MarkerObservation {
    cv::Point2d position; // 2D sub-pixel position in the image
    double intensity;     // peak or average brightness of the glint
};

#include <string>

struct VisionDiagnostics {
    struct Candidate {
        cv::Point2d centroid;
        cv::Rect boundingBox;
        double area = 0.0;
        double circularity = 0.0;
        bool isOverlapping = false;
        bool accepted = false;
        std::string reason; // "Area too small", "Low circularity", "Accepted", etc.
        std::vector<cv::Point2d> markers;
    };
    std::vector<Candidate> candidates;
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
    virtual std::vector<BallObservation> detectBalls(const cv::Mat& frame, VisionDiagnostics* diag) {
        return detectBalls(frame);
    }
};

