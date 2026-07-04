#pragma once
#include "Math/IKinematicsSolver.hpp"

class EigenBallisticsEngine : public IKinematicsSolver {
public:
    LaunchData<Degrees, MilesPerHour> solveKinematics(
        const std::vector<Ball3D>& trajectory,
        double pulseIntervalMs
    ) override;
};

