#pragma once
// Mock pipeline components for testing SessionStateMachine in isolation.
// State is `static inline` so a test can script behaviour before constructing
// the state machine and inspect results afterwards.
#include "Math/IComputerVision.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Math/LaunchData.hpp"
#include "Math/Units.hpp"
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

struct MockStrobeTrigger {
    static inline bool fireOnNext = false;
    bool checkTrigger(const cv::Mat&, const cv::Mat&) {
        if (fireOnNext) {
            fireOnNext = false;
            return true;
        }
        return false;
    }
    void reset() { fireOnNext = false; }
};

struct MockStrobeVision : public IComputerVision {
    static inline std::vector<BallObservation> returnObs;
    std::vector<BallObservation> detectBalls(const cv::Mat&) override {
        return returnObs;
    }
};

struct MockStrobeSpatial : public ISpatialSolver {
    static inline std::vector<Ball3D> returnBalls;
    std::vector<Ball3D> triangulateShot(
        const std::vector<BallObservation>&,
        const std::vector<BallObservation>&
    ) override {
        return returnBalls;
    }
};

struct MockStrobeKinematics : public IKinematicsSolver {
    static inline double lastPulseIntervalMs = 0.0;
    static inline size_t lastTrajectorySize = 0;
    LaunchData<Degrees, MilesPerHour> solveKinematics(const std::vector<Ball3D>& trajectory, double pulseIntervalMs) override {
        lastTrajectorySize = trajectory.size();
        lastPulseIntervalMs = pulseIntervalMs;
        LaunchData<Degrees, MilesPerHour> ld;
        ld.ballSpeed = MilesPerHour(85.0);
        ld.verticalLaunchAngle = Degrees(14.0);
        ld.horizontalLaunchAngle = Degrees(1.0);
        ld.spinRPM = 6500.0;
        ld.spinAxis = Eigen::Vector3d(0, 1, 0);
        return ld;
    }
};

struct MockStrobeNet : public INetworkTransmitter {
    static inline bool transmitted = false;
    bool transmitLaunchData(const LaunchData<Degrees, MilesPerHour>&) override {
        transmitted = true;
        return true;
    }
};
