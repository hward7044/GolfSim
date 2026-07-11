#include "Math/Units.hpp"
#include "Math/OpticalGateTrigger.hpp"
#include "Math/OpenCVMomentsTracker.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/EigenBallisticsEngine.hpp"
#include "Diagnostics/FlightRecorder.hpp"
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>
#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>
#include <filesystem>
#include <thread>
#include <numbers>


void testUnits() {
    MetersPerSecond mps(10.0);
    MilesPerHour mph = to_mph(mps);
    assert(std::abs(mph.value() - 22.36936) < 1e-4);

    Radians rad(std::numbers::pi / 4.0);
    Degrees deg = to_degrees(rad);
    assert(std::abs(deg.value() - 45.0) < 1e-4);

    Degrees deg2(90.0);
    Radians rad2 = to_radians(deg2);
    assert(std::abs(rad2.value() - std::numbers::pi / 2.0) < 1e-6);

    MetersPerSecond mps2 = to_mps(mph);
    assert(std::abs(mps2.value() - 10.0) < 1e-5);

    spdlog::info("[TEST] Units verification passed.");
}

void testOpticalGateTrigger() {
    cv::Rect roi(10, 10, 50, 50);
    OpticalGateTrigger trigger(roi, 100, 15, 0.1);

    // Frame 1: Blank frame (all black)
    cv::Mat frame1 = cv::Mat::zeros(100, 100, CV_8UC1);
    bool trig1 = trigger.checkOpticalGate(frame1);
    assert(!trig1); // First frame should initialize background

    // Frame 2: Still blank
    bool trig2 = trigger.checkOpticalGate(frame1);
    assert(!trig2); // No difference

    // Frame 3: Draw a bright square inside the ROI (representing a ball)
    cv::Mat frame3 = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::rectangle(frame3, cv::Rect(20, 20, 20, 20), cv::Scalar(255), -1); // 400 white pixels
    bool trig3 = trigger.checkOpticalGate(frame3);
    assert(trig3); // Should trigger (400 > 100 min pixels)

    // Check diagnostics
    nlohmann::json diag = trigger.getLatestDiagnostics();
    assert(diag["triggered"] == trig3);
    assert(diag["nonZeroCount"].get<int>() > 0);
    assert(diag["minBallPixels"] == 100);
    assert(diag["pixelDiffThreshold"] == 15);
    assert(diag["gateROI"][2] == 50); // width

    spdlog::info("[TEST] OpticalGateTrigger verification passed.");
}

void testOpenCVMomentsTracker() {
    OpenCVMomentsTracker tracker(50, 200, 50, 10000, 0.5);

    // Create frame with a simulated ball (circle of radius 15) and a single marker inside it
    cv::Mat frame = cv::Mat::zeros(400, 400, CV_8UC1);
    cv::Point center(200, 200);
    cv::circle(frame, center, 15, cv::Scalar(100), -1); // Ball silhouette (diffuse intensity 100)
    cv::circle(frame, cv::Point(205, 195), 2, cv::Scalar(255), -1); // Marker glint (intensity 255)
    
    // Draw a small noise spot that will be rejected due to area < 50
    cv::circle(frame, cv::Point(20, 20), 1, cv::Scalar(100), -1);

    auto balls = tracker.detectBalls(frame);
    assert(balls.size() == 1);
    assert(std::abs(balls[0].centroid.x - 200) < 1.0);
    assert(std::abs(balls[0].centroid.y - 200) < 1.0);
    assert(balls[0].markers.size() == 1);
    assert(std::abs(balls[0].markers[0].position.x - 205) < 1.0);
    assert(std::abs(balls[0].markers[0].position.y - 195) < 1.0);

    // Verify diagnostics
    nlohmann::json vdiag = tracker.getLatestDiagnostics();
    assert(vdiag["candidates"].size() >= 2);
    bool foundBall = false;
    bool foundNoise = false;
    for (const auto& cand : vdiag["candidates"]) {
        bool accepted = cand.value("accepted", false);
        std::string reason = cand.value("reason", "");
        if (accepted) {
            auto cen = cand["centroid"];
            assert(std::abs(cen[0].get<double>() - 200) < 1.0);
            assert(reason == "Accepted (Moments)");
            assert(cand["markers"].size() == 1);
            assert(std::abs(cand["markers"][0][0].get<double>() - 205) < 1.0);
            foundBall = true;
        } else {
            assert(reason == "Area too small");
            foundNoise = true;
        }
    }
    assert(foundBall);
    assert(foundNoise);

    spdlog::info("[TEST] OpenCVMomentsTracker verification passed.");
}

void testStereoTriangulatorAndRaySphere() {
    StereoCalibration calib;
    // Let's set up a standard horizontal camera setup.
    // Focal length = 1000 pixels. Center = (640, 400).
    calib.K_L = (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 640.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 1.0);
    calib.D_L = cv::Mat::zeros(1, 5, CV_64F);
    calib.K_R = calib.K_L.clone();
    calib.D_R = calib.D_L.clone();
    calib.R = cv::Mat::eye(3, 3, CV_64F);
    calib.T = (cv::Mat_<double>(3, 1) << -0.1, 0.0, 0.0); // 100mm baseline along X

    calib.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib.R_R = cv::Mat::eye(3, 3, CV_64F);

    // Rectification projection matrices
    calib.P_L = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    calib.P_R = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, -100.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    StereoTriangulator solver(calib, 0.021335);

    // Let's place a ball at world coords: (0.0, 0.0, 1.5) meters (1.5 meters directly in front of left camera)
    // Left projection:
    // X_norm = 0 / 1.5 = 0 -> u = 640
    // Y_norm = 0 / 1.5 = 0 -> v = 400
    // Right projection:
    // Right camera center is at (-0.1, 0.0, 0.0) relative to Left.
    // So point in Right camera frame is (0.1, 0.0, 1.5).
    // X_norm_R = 0.1 / 1.5 -> u_R = 1000 * (0.1 / 1.5) + 640 = 706.667
    // Y_norm_R = 0.0 / 1.5 -> v_R = 400

    BallObservation bL, bR;
    bL.centroid = cv::Point2d(640.0, 400.0);
    bR.centroid = cv::Point2d(573.333333, 400.0);

    // Put a marker at (0.0, 0.021335, 1.5) in world (on the ball's top surface)
    // Left projection: X=0, Y=0.021335, Z=1.5
    // u = 1000 * 0 + 640 = 640
    // v = 1000 * (0.021335 / 1.5) + 400 = 414.2233
    // Right projection: X=-0.1, Y=0.021335, Z=1.5 (since Right camera center is at T_x = -0.1 relative to Left)
    // u_R = 1000 * (-0.1 / 1.5) + 640 = 573.3333
    // v_R = 1000 * (0.021335 / 1.5) + 400 = 414.2233
    MarkerObservation ml, mr;
    ml.position = cv::Point2d(640.0, 414.2233);
    mr.position = cv::Point2d(573.333333, 414.2233);

    bL.markers.push_back(ml);
    bR.markers.push_back(mr);

    auto result3D = solver.triangulateShot({ bL }, { bR });
    assert(result3D.size() == 1);
    assert(std::abs(result3D[0].centroid.z() - 1.5) < 1e-3);
    assert(std::abs(result3D[0].centroid.x() - 0.0) < 1e-3);
    assert(std::abs(result3D[0].centroid.y() - 0.0) < 1e-3);

    assert(result3D[0].markers.size() == 1);
    assert(result3D[0].markers[0].isStereo);
    assert(std::abs(result3D[0].markers[0].position.y() - 0.021335) < 1e-3);

    // Let's test the single-camera ray-sphere fallback.
    // If the marker is only visible in Left camera, we remove it from Right:
    bR.markers.clear();
    auto resultSingle = solver.triangulateShot({ bL }, { bR });
    assert(resultSingle.size() == 1);
    assert(resultSingle[0].markers.size() == 1);
    assert(!resultSingle[0].markers[0].isStereo);
    assert(resultSingle[0].markers[0].confidence == 0.5);
    // The recovered position should be extremely close to (0.0, 0.021335, 1.5)
    assert(std::abs(resultSingle[0].markers[0].position.y() - 0.021335) < 1e-3);
    assert(std::abs(resultSingle[0].markers[0].position.z() - 1.5) < 1e-3);

    spdlog::info("[TEST] StereoTriangulator and Ray-Sphere Fallback verification passed.");
}

void testKinematicsEngine() {
    EigenBallisticsEngine engine;

    // Create a synthetic shot trajectory.
    // 3 pulses, spacing T = 2ms.
    // Camera velocity: vx = 50 m/s (~112 mph) downrange,
    //                  vy = -10 m/s (upward in world space),
    //                  vz = 2 m/s (slight lateral depth).
    // Constant Spin: Backspin (spin axis is negative Z in camera space) at 3000 RPM.
    // 3000 RPM = 50 rev/sec = 50 * 2pi rad/sec = 100pi rad/sec.
    // In dt = 0.002s, rotation angle = 100pi * 0.002 = 0.2pi rad (~36 degrees).
    double dt = 0.002;
    double spin_speed = 3000.0 * (2.0 * std::numbers::pi) / 60.0; // ~314.159 rad/s
    Eigen::Vector3d axis(0.0, 0.0, -1.0); // Z-axis (backspin in camera frame)

    std::vector<Ball3D> trajectory;
    Eigen::Vector3d start_centroid(0.0, 0.0, 1.0);
    Eigen::Vector3d velocity(50.0, -10.0, 2.0);

    // Local marker positions on ball surface at t=0
    std::vector<Eigen::Vector3d> local_markers = {
        Eigen::Vector3d(0.021335, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 0.021335)
    };

    for (int k = 0; k < 4; ++k) {
        double tk = k * dt;
        Ball3D ball;
        ball.centroid = start_centroid + velocity * tk;

        // Apply rotation to local markers
        double theta = spin_speed * tk;
        Eigen::AngleAxisd R_rot(theta, axis);
        
        for (const auto& lm : local_markers) {
            Marker3D m;
            m.position = ball.centroid + R_rot * lm;
            m.confidence = 1.0;
            m.isStereo = true;
            ball.markers.push_back(m);
        }
        trajectory.push_back(ball);
    }

    auto data = engine.solveKinematics(trajectory, 2.0);
    
    // Verify velocity and speed
    double expected_speed_mps = velocity.norm();
    double expected_speed_mph = expected_speed_mps * 2.236936;
    assert(std::abs(data.ballSpeed.value() - expected_speed_mph) < 1e-2);

    // Verify launch angles using the corrected physical equations:
    // VLA = atan2(-vy, sqrt(vx^2 + vz^2))
    // HLA = atan2(vz, vx)
    double expected_vla = std::atan2(-velocity.y(), std::sqrt(velocity.x()*velocity.x() + velocity.z()*velocity.z())) * 180.0 / std::numbers::pi;
    double expected_hla = std::atan2(velocity.z(), velocity.x()) * 180.0 / std::numbers::pi;
    assert(std::abs(data.verticalLaunchAngle.value() - expected_vla) < 1e-2);
    assert(std::abs(data.horizontalLaunchAngle.value() - expected_hla) < 1e-2);

    // Verify spin speed and axis (solver negates Y component of spin axis for world coords)
    assert(std::abs(data.spinRPM - 3000.0) < 1.0);
    Eigen::Vector3d expected_world_spin_axis(axis.x(), -axis.y(), axis.z());
    assert((data.spinAxis - expected_world_spin_axis.normalized()).norm() < 1e-3);

    spdlog::info("[TEST] EigenBallisticsEngine verification passed.");
}

void testFlightRecorder() {
    FlightRecorder recorder("build/replays_test");

    // Clean up test dir if it exists
    std::filesystem::remove_all("build/replays_test");

    std::vector<RecordedFrame> frames;
    for (int i = 0; i < 12; ++i) {
        RecordedFrame f;
        f.timestamp = i * 1000;
        f.leftFrame = cv::Mat::zeros(100, 100, CV_8UC1);
        f.rightFrame = cv::Mat::zeros(100, 100, CV_8UC1);
        
        f.triggerDiag = {
            {"triggered", false},
            {"nonZeroCount", 10},
            {"minBallPixels", 100},
            {"pixelDiffThreshold", 20},
            {"gateROI", {10, 10, 50, 50}}
        };

        f.leftVisionDiag = {
            {"candidates", {
                {
                    {"centroid", {50.0, 50.0}},
                    {"boundingBox", {40, 40, 20, 20}},
                    {"area", 400.0},
                    {"circularity", 1.0},
                    {"isOverlapping", false},
                    {"accepted", true},
                    {"reason", "Accepted (Moments)"},
                    {"markers", {{45.0, 45.0}}}
                }
            }}
        };
        f.rightVisionDiag = f.leftVisionDiag;

        frames.push_back(f);
    }

    LaunchData<Degrees, MilesPerHour> launchData;
    launchData.ballSpeed = MilesPerHour(100.0);
    launchData.verticalLaunchAngle = Degrees(15.0);
    launchData.horizontalLaunchAngle = Degrees(2.0);
    launchData.spinRPM = 3000.0;
    launchData.spinAxis = Eigen::Vector3d(0, 0, -1);

    // Let's call saveSession 12 times to trigger limit rotation (which caps at 10)
    for (int i = 0; i < 12; ++i) {
        recorder.saveSession(frames, launchData);
        // sleep a tiny bit to ensure distinct ms timestamps
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Wait a little for async writes to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    // Check that we have exactly 10 directories in build/replays_test starting with shot_
    int count = 0;
    for (const auto& entry : std::filesystem::directory_iterator("build/replays_test")) {
        if (entry.is_directory() && entry.path().filename().string().rfind("shot_", 0) == 0) {
            count++;
            // Check that subdirectories raw/ and annotated/ and metadata.json exist
            assert(std::filesystem::exists(entry.path() / "raw"));
            assert(std::filesystem::exists(entry.path() / "annotated"));
            assert(std::filesystem::exists(entry.path() / "metadata.json"));
        }
    }
    assert(count == 10);

    // Clean up
    std::filesystem::remove_all("build/replays_test");
    spdlog::info("[TEST] FlightRecorder verification passed.");
}

void runMathTests() {
    spdlog::info("============================================");
    spdlog::info("Starting C++ Math Verification Tests...");
    spdlog::info("============================================");

    testUnits();
    testOpticalGateTrigger();
    testOpenCVMomentsTracker();
    testStereoTriangulatorAndRaySphere();
    testKinematicsEngine();
    testFlightRecorder();

    spdlog::info("============================================");
    spdlog::info("All C++ Math Verification Tests PASSED!");
    spdlog::info("============================================");
}
