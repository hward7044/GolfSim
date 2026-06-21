#pragma once
#include <Eigen/Core>

// T.4 — express syntax tree / strong types
// template<typename AngleUnit, typename SpeedUnit>
// Units enforced at compile time; implicit conversions are errors.
template<typename AngleUnit, typename SpeedUnit>
struct LaunchData {
    SpeedUnit  ballSpeed;
    AngleUnit  verticalLaunchAngle;
    AngleUnit  horizontalLaunchAngle;
    double     spinRPM;
    Eigen::Vector3d spinAxis;
};
