#pragma once
#include "Math/IComputerVision.hpp"
#include <Eigen/Core>
#include <vector>

struct Marker3D {
    Eigen::Vector3d position; // Reconstructed 3D position in world space
    double confidence;        // 1.0 for stereo-triangulated, 0.5 for single-camera fallback
    bool isStereo;            // true if visible in both cameras
};

struct Ball3D {
    Eigen::Vector3d centroid; // Reconstructed 3D ball centroid in world space
    std::vector<Marker3D> markers; // 3D reconstructed markers for this ball
};

class ISpatialSolver {
public:
    virtual ~ISpatialSolver() = default;
    virtual std::vector<Ball3D> triangulateShot(
        const std::vector<BallObservation>& leftObs,
        const std::vector<BallObservation>& rightObs
    ) = 0;
};

