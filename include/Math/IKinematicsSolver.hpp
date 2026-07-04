#pragma once
#include "Math/ISpatialSolver.hpp"
#include "Math/LaunchData.hpp"

class IKinematicsSolver {
public:
    virtual ~IKinematicsSolver() = default;
    
    virtual LaunchData<Degrees, MilesPerHour> solveKinematics(
        const std::vector<Ball3D>& trajectory,
        double pulseIntervalMs
    ) = 0;
};

