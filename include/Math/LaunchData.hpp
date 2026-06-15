#pragma once
#include <Eigen/Core>
class LaunchData {
public:
    double ballSpeed;
    double verticalLaunchAngle;
    double horizontalLaunchAngle;
    double spinRPM;
    Eigen::Vector3d spinAxis;
};
