#include "Math/Units.hpp"
#include "Math/OpticalGateTrigger.hpp"
#include "Math/BallPresenceTrigger.hpp"
#include "Math/StereoBallTrackerTrigger.hpp"
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

void testBallPresenceTrigger() {
    cv::Rect teeRoi(10, 10, 80, 80);
    BallPresenceTrigger trigger(teeRoi, 5, 50, 50, 1000, 0.5, 0.5);

    // Frame with a stable circular ball (radius 15) inside tee ROI
    cv::Mat ballFrame = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::circle(ballFrame, cv::Point(50, 50), 15, cv::Scalar(200), -1);

    // Feed for 4 frames (under stability threshold 5)
    for (int i = 0; i < 4; ++i) {
        bool trig = trigger.checkOpticalGate(ballFrame);
        assert(!trig);
        nlohmann::json diag = trigger.getLatestDiagnostics();
        assert(diag["state"] == "WAITING_FOR_BALL");
        assert(diag["stabilityCounter"] == i + 1);
    }

    // 5th frame reaches stability threshold -> BALL_LOCKED
    bool trigLock = trigger.checkOpticalGate(ballFrame);
    assert(!trigLock);
    nlohmann::json diagLocked = trigger.getLatestDiagnostics();
    assert(diagLocked["state"] == "BALL_LOCKED");

    // Shadow Test: Dim the ball intensity by 30% (simulating hand/club shadow or IR fluctuation)
    cv::Mat dimmedFrame = cv::Mat::zeros(100, 100, CV_8UC1);
    cv::circle(dimmedFrame, cv::Point(50, 50), 15, cv::Scalar(140), -1);
    bool trigShadow = trigger.checkOpticalGate(dimmedFrame);
    assert(!trigShadow); // SHADOW IMMUNITY PASSED: Must NOT trigger false departure!
    nlohmann::json diagShadow = trigger.getLatestDiagnostics();
    assert(diagShadow["state"] == "BALL_LOCKED");
    assert(diagShadow["matchScore"].get<float>() > 0.70f);

    // Departure Test: Feed black frame (ball physically departed from tee)
    cv::Mat blankFrame = cv::Mat::zeros(100, 100, CV_8UC1);
    bool trigDeparted = trigger.checkOpticalGate(blankFrame);
    assert(trigDeparted); // Triggered! Ball pixel pattern vanished.
    nlohmann::json diagDeparted = trigger.getLatestDiagnostics();
    assert(diagDeparted["state"] == "BALL_DEPARTED");

    // Test reset
    trigger.reset();
    nlohmann::json diagReset = trigger.getLatestDiagnostics();
    assert(diagReset["state"] == "WAITING_FOR_BALL");
    assert(diagReset["stabilityCounter"] == 0);

    spdlog::info("[TEST] BallPresenceTrigger verification passed.");
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

    // Draw a thin rectangle (area ~100) that has low circularity (< 0.5) to test circularity rejection
    cv::rectangle(frame, cv::Rect(300, 50, 4, 25), cv::Scalar(100), -1);

    auto balls = tracker.detectBalls(frame);
    assert(balls.size() == 1);
    assert(std::abs(balls[0].centroid.x - 200) < 1.0);
    assert(std::abs(balls[0].centroid.y - 200) < 1.0);
    assert(balls[0].markers.size() == 1);
    assert(std::abs(balls[0].markers[0].position.x - 205) < 1.0);
    assert(std::abs(balls[0].markers[0].position.y - 195) < 1.0);

    // Verify diagnostics
    nlohmann::json vdiag = tracker.getLatestDiagnostics();
    assert(vdiag["candidates"].size() >= 3);
    bool foundBall = false;
    bool foundNoiseArea = false;
    bool foundNoiseCirc = false;
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
        } else if (reason == "Area too small") {
            foundNoiseArea = true;
        } else if (reason == "Circularity too low") {
            foundNoiseCirc = true;
        }
    }
    assert(foundBall);
    assert(foundNoiseArea);
    assert(foundNoiseCirc);

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

    // Wait for async worker thread to finish writing files
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

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

void testStereoBallTrackerTrigger() {
    StereoCalibration calib;
    calib.K_L = (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 640.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 1.0);
    calib.D_L = cv::Mat::zeros(1, 5, CV_64F);
    calib.K_R = calib.K_L.clone();
    calib.D_R = calib.D_L.clone();
    calib.R = cv::Mat::eye(3, 3, CV_64F);
    calib.T = (cv::Mat_<double>(3, 1) << -0.1, 0.0, 0.0); // 100mm baseline
    calib.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib.R_R = cv::Mat::eye(3, 3, CV_64F);
    calib.P_L = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    calib.P_R = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, -100.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // Instantiate tracker with wide search ROI and 2ft parameters
    cv::Rect searchRoiL(400, 200, 480, 400);
    cv::Rect searchRoiR(400, 200, 480, 400);
    StereoBallTrackerTrigger trigger(calib, searchRoiL, searchRoiR, 25.0, 50.0, 0.60, 100, 3.0, 256, 4, 4.0, 0.04);

    // Frame setup for ball at (0, 0, 0.6) m (2 ft distance)
    // f = 1000, B = 0.1m => Disparity d = (f * B) / Z = 100 / 0.6 = 166.67 px
    // Left center (640, 400), Right center (640 - 167 = 473, 400)
    // Radius at 0.6m = 1000 * (0.021335 / 0.6) = 35.56 px -> radius = 35 px
    cv::Mat frameL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(frameL, cv::Point(640, 400), 35, cv::Scalar(200), -1);
    cv::circle(frameR, cv::Point(473, 400), 35, cv::Scalar(200), -1);

    // 1. Test SEARCHING -> ARMED Transition
    bool trig1 = trigger.checkTrigger(frameL, frameR);
    assert(!trig1);
    assert(trigger.getState() == StereoTriggerState::ARMED);
    auto pos3D = trigger.getLastKnown3DPosition();
    assert(std::abs(pos3D.z() - 0.6) < 0.02);

    // 2. Test SINGLE-CAMERA OCCLUSION IMMUNITY (hand behind ball)
    cv::Mat occludedFrameL = frameL.clone();
    cv::rectangle(occludedFrameL, cv::Rect(600, 350, 100, 100), cv::Scalar(180), -1);
    cv::Mat blankFrameR = cv::Mat::zeros(800, 1280, CV_8UC1);

    bool trigOcc = trigger.checkTrigger(occludedFrameL, blankFrameR);
    assert(!trigOcc);
    assert(trigger.getState() == StereoTriggerState::ARMED);

    // 3. Test DUAL-CAMERA LOSS GRACE WINDOW
    for (int i = 0; i < 3; ++i) {
        bool trigGrace = trigger.checkTrigger(blankFrameR, blankFrameR);
        assert(!trigGrace);
        assert(trigger.getState() == StereoTriggerState::ARMED);
    }
    bool trigReset = trigger.checkTrigger(blankFrameR, blankFrameR);
    assert(!trigReset);
    assert(trigger.getState() == StereoTriggerState::SEARCHING);

    // Re-lock ball to ARMED
    trigger.checkTrigger(frameL, frameR);
    assert(trigger.getState() == StereoTriggerState::ARMED);

    // 4. Test VIBRATION / NUDGE REJECTION
    // Displace ball slightly by 45 mm (beyond 40 mm threshold), but low speed (0.5 m/s)
    // At Z=0.6m, 45mm horizontal displacement = ~75 px -> u_L = 640 + 75 = 715, u_R = 473 + 75 = 548
    cv::Mat nudgeL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat nudgeR = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(nudgeL, cv::Point(715, 400), 35, cv::Scalar(200), -1);
    cv::circle(nudgeR, cv::Point(548, 400), 35, cv::Scalar(200), -1);

    bool trigNudge = trigger.checkTrigger(nudgeL, nudgeR);
    assert(!trigNudge);
    assert(trigger.getState() == StereoTriggerState::CONFIRMING);

    for (int i = 0; i < 3; ++i) {
        bool trigConfirm = trigger.checkTrigger(nudgeL, nudgeR);
        assert(!trigConfirm);
    }
    assert(trigger.getState() == StereoTriggerState::ARMED);

    // 5. Test VALID HIGH-VELOCITY IMPACT TRIGGER
    // Displace ball at launch speed 40 m/s (~90 mph)
    cv::Mat launchL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat launchR = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(launchL, cv::Point(800, 400), 35, cv::Scalar(200), -1);
    cv::circle(launchR, cv::Point(633, 400), 35, cv::Scalar(200), -1);

    bool trigImpact1 = trigger.checkTrigger(launchL, launchR);
    assert(!trigImpact1);
    assert(trigger.getState() == StereoTriggerState::CONFIRMING);

    cv::Mat launchL2 = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat launchR2 = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(launchL2, cv::Point(900, 400), 35, cv::Scalar(200), -1);
    cv::circle(launchR2, cv::Point(733, 400), 35, cv::Scalar(200), -1);

    bool trigImpact2 = trigger.checkTrigger(launchL2, launchR2);
    assert(trigImpact2);
    assert(trigger.getState() == StereoTriggerState::CAPTURED);

    spdlog::info("[TEST] StereoBallTrackerTrigger verification passed.");
}

void runMathTests() {
    spdlog::info("============================================");
    spdlog::info("Starting C++ Math Verification Tests...");
    spdlog::info("============================================");

    testUnits();
    testBallPresenceTrigger();
    testStereoBallTrackerTrigger();
    testOpenCVMomentsTracker();
    testStereoTriangulatorAndRaySphere();
    testKinematicsEngine();
    testFlightRecorder();

    spdlog::info("============================================");
    spdlog::info("All C++ Math Verification Tests PASSED!");
    spdlog::info("============================================");
}
