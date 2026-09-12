#include "Math/Units.hpp"
#include "Math/BallPresenceTrigger.hpp"
#include "Math/StereoBallTrackerTrigger.hpp"
#include "Math/OpenCVMomentsTracker.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/EigenBallisticsEngine.hpp"
#include "Math/AtomicRingBuffer.hpp"
#include "Camera/FrameSet.hpp"
#include "Diagnostics/FlightRecorder.hpp"
#include "Orchestration/SessionStateMachine.hpp"
#include "Orchestration/PipelineTimingConfig.hpp"
#include "HAL/SerialPort.hpp"
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
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
    assert(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);

    // Emitter Protection Test: Fast loss timeout (50ms)
    trigger.setLossTimeoutSec(0.05); // 50 ms test timeout
    trigger.checkOpticalGate(blankFrame); // Starts empty timer
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    trigger.checkOpticalGate(blankFrame); // Triggers timeout
    assert(trigger.getEmitterMode() == EmitterPowerMode::LOW_STANDBY);
    assert(trigger.isStandbyRequested());
    nlohmann::json diagStandby = trigger.getLatestDiagnostics();
    assert(diagStandby["emitterMode"] == "STANDBY");

    // Restoring ball on tee immediately wakes up to HIGH_STROBE_READY
    trigger.checkOpticalGate(ballFrame);
    assert(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);
    assert(!trigger.isStandbyRequested());

    // Verify ITriggerDetector interface polymorphism and zero recursion
    ITriggerDetector* baseDetector = &trigger;
    bool trigPoly1 = baseDetector->checkTrigger(ballFrame, ballFrame);
    bool trigPoly2 = baseDetector->checkOpticalGate(ballFrame);
    (void)trigPoly1;
    (void)trigPoly2;

    spdlog::info("[TEST] BallPresenceTrigger verification passed (including dynamic emitter protection).");
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
    calib.K_L = cv::Mat_<double>({3, 3}, {1000.0, 0.0, 640.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 1.0});
    calib.D_L = cv::Mat::zeros(1, 5, CV_64F);
    calib.K_R = calib.K_L.clone();
    calib.D_R = calib.D_L.clone();
    calib.R = cv::Mat::eye(3, 3, CV_64F);
    calib.T = cv::Mat_<double>({3, 1}, {-0.1, 0.0, 0.0}); // 100mm baseline along X

    calib.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib.R_R = cv::Mat::eye(3, 3, CV_64F);

    // Rectification projection matrices
    calib.P_L = cv::Mat_<double>({3, 4}, {1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0});
    calib.P_R = cv::Mat_<double>({3, 4}, {1000.0, 0.0, 640.0, -100.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0});

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

    // Test single-camera ray-sphere fallback for Right camera:
    // If marker is only visible in Right camera, remove it from Left:
    bL.markers.clear();
    bR.markers.push_back(mr);
    auto resultRightOnly = solver.triangulateShot({ bL }, { bR });
    assert(resultRightOnly.size() == 1);
    assert(resultRightOnly[0].markers.size() == 1);
    assert(!resultRightOnly[0].markers[0].isStereo);
    assert(resultRightOnly[0].markers[0].confidence == 0.5);
    // The recovered position in world (Left camera) coords must match (0.0, 0.021335, 1.5)
    assert(std::abs(resultRightOnly[0].markers[0].position.x() - 0.0) < 1e-3);
    assert(std::abs(resultRightOnly[0].markers[0].position.y() - 0.021335) < 1e-3);
    assert(std::abs(resultRightOnly[0].markers[0].position.z() - 1.5) < 1e-3);

    // Test Principal Motion Vector Trajectory Sorting:
    // Create a steep vertical launch shot (lob wedge):
    // Pulse 0 (lowest in air): Left u = 640.0, v = 466.67 | Right u = 573.33, v = 466.67
    // Pulse 1 (mid air): Left u = 638.0 (slight horizontal jitter), v = 400.00 | Right u = 571.33, v = 400.00
    // Pulse 2 (highest in air): Left u = 666.67, v = 333.33 | Right u = 600.00, v = 333.33
    BallObservation p0_L, p0_R, p1_L, p1_R, p2_L, p2_R;
    p0_L.centroid = cv::Point2d(640.0, 466.67);
    p0_R.centroid = cv::Point2d(573.333333, 466.67);

    p1_L.centroid = cv::Point2d(638.0, 400.00);
    p1_R.centroid = cv::Point2d(571.333333, 400.00);

    p2_L.centroid = cv::Point2d(666.67, 333.33);
    p2_R.centroid = cv::Point2d(600.00, 333.33);

    // Pass them intentionally out-of-order: { p2, p0, p1 }
    auto steepTrajectory = solver.triangulateShot({ p2_L, p0_L, p1_L }, { p2_R, p0_R, p1_R });
    assert(steepTrajectory.size() == 3);
    // Principal motion vector projection must restore exact chronological order: p0 -> p1 -> p2
    // In world coordinates, +Y is down, so p0 (lowest) has highest Y, p2 (highest) has lowest Y.
    assert(steepTrajectory[0].centroid.y() > steepTrajectory[1].centroid.y());
    assert(steepTrajectory[1].centroid.y() > steepTrajectory[2].centroid.y());

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
    calib.K_L = cv::Mat_<double>({3, 3}, {1000.0, 0.0, 640.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 1.0});
    calib.D_L = cv::Mat::zeros(1, 5, CV_64F);
    calib.K_R = calib.K_L.clone();
    calib.D_R = calib.D_L.clone();
    calib.R = cv::Mat::eye(3, 3, CV_64F);
    calib.T = cv::Mat_<double>({3, 1}, {-0.1, 0.0, 0.0}); // 100mm baseline
    calib.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib.R_R = cv::Mat::eye(3, 3, CV_64F);
    calib.P_L = cv::Mat_<double>({3, 4}, {1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0});
    calib.P_R = cv::Mat_<double>({3, 4}, {1000.0, 0.0, 640.0, -100.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0});

    // Instantiate tracker with wide search ROI, 5-frame stability lock, and 3ft (0.9144m) limit
    cv::Rect searchRoiL(400, 200, 480, 400);
    cv::Rect searchRoiR(400, 200, 480, 400);
    StereoBallTrackerTrigger trigger(calib, searchRoiL, searchRoiR, 5.0, 50.0, 0.50, 100, 30.0, 256, 0.9144, 5, 4, 4.0, 0.04);

    // Frame setup for ball at (0, 0, 0.6) m (2 ft distance)
    // f = 1000, B = 0.1m => Disparity d = (f * B) / Z = 100 / 0.6 = 166.67 px
    // Left center (640, 400), Right center (640 - 167 = 473, 400)
    // Radius at 0.6m = 1000 * (0.021335 / 0.6) = 35.56 px -> radius = 35 px
    cv::Mat frameL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(frameL, cv::Point(640, 400), 35, cv::Scalar(200), -1);
    cv::circle(frameR, cv::Point(473, 400), 35, cv::Scalar(200), -1);

    // 0. Test 3FT DISTANCE REJECTION (Ball placed at Z = 1.0m / 3.28 ft > 3.0 ft)
    // f = 1000, B = 0.1m => d = 100/1.0 = 100 px => Right center = 640 - 100 = 540
    cv::Mat farFrameL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat farFrameR = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(farFrameL, cv::Point(640, 400), 21, cv::Scalar(200), -1);
    cv::circle(farFrameR, cv::Point(540, 400), 21, cv::Scalar(200), -1);

    for (int i = 0; i < 6; ++i) {
        bool trigFar = trigger.checkTrigger(farFrameL, farFrameR);
        assert(!trigFar);
        assert(trigger.getState() == StereoTriggerState::SEARCHING);
    }

    // 1. Test SEARCHING -> ARMED Transition (Requires 5 consecutive stable frames)
    for (int i = 0; i < 4; ++i) {
        bool trigSearch = trigger.checkTrigger(frameL, frameR);
        assert(!trigSearch);
        assert(trigger.getState() == StereoTriggerState::SEARCHING);
    }
    // 5th frame transitions state to ARMED
    bool trigArmed = trigger.checkTrigger(frameL, frameR);
    assert(!trigArmed);
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

    // Re-lock ball to ARMED (requires 5 frames)
    for (int i = 0; i < 5; ++i) {
        trigger.checkTrigger(frameL, frameR);
    }
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

    // 6. Test STANDBY EMITTER PROTECTION (Dynamic Photobiological Safety)
    trigger.reset();
    assert(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);
    assert(!trigger.isStandbyRequested());

    // Set fast test timeout (50ms)
    trigger.setLossTimeoutSec(0.05);
    trigger.checkTrigger(blankFrameR, blankFrameR); // Starts empty timer
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    trigger.checkTrigger(blankFrameR, blankFrameR); // Triggers transition to LOW_STANDBY
    assert(trigger.getEmitterMode() == EmitterPowerMode::LOW_STANDBY);
    assert(trigger.isStandbyRequested());

    // Placing ball back on tee restores HIGH_STROBE_READY
    trigger.checkTrigger(frameL, frameR);
    assert(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);
    assert(!trigger.isStandbyRequested());

    spdlog::info("[TEST] StereoBallTrackerTrigger verification passed (including dynamic emitter protection).");
}

void testAtomicRingBufferOverwrite() {
    AtomicRingBuffer<FrameSet, 16> ringBuffer;
    ringBuffer.preallocate(100, 100);

    assert(ringBuffer.empty());
    assert(ringBuffer.size() == 0);
    assert(ringBuffer.capacity() == 15);

    // 1. Test basic push and pop
    FrameSet pushFrame;
    pushFrame.preallocate(100, 100);
    pushFrame.timestamp = 1001;

    ringBuffer.push(pushFrame);
    assert(!ringBuffer.empty());
    assert(ringBuffer.size() == 1);

    FrameSet popFrame;
    popFrame.preallocate(100, 100);
    bool popSuccess = ringBuffer.pop(popFrame);
    assert(popSuccess);
    assert(popFrame.timestamp == 1001);
    assert(popFrame.getFrame(CameraRole::STEREO_LEFT).rows == 100);
    assert(popFrame.getFrame(CameraRole::STEREO_LEFT).cols == 100);
    assert(ringBuffer.empty());

    // 2. Test overwrite semantics (pushing 30 items into capacity 15 buffer)
    for (uint64_t i = 0; i < 30; ++i) {
        FrameSet f;
        f.preallocate(100, 100);
        f.timestamp = i;
        ringBuffer.push(f);
    }

    // Since capacity is 15, size should be <= 15
    assert(ringBuffer.size() <= 15);

    // Drain and verify monotonic increasing timestamps and no corrupted frames
    uint64_t lastTimestamp = 0;
    size_t drainedCount = 0;
    FrameSet drainedFrame;
    drainedFrame.preallocate(100, 100);
    while (ringBuffer.pop(drainedFrame)) {
        assert(drainedFrame.getFrame(CameraRole::STEREO_LEFT).rows == 100);
        assert(drainedFrame.getFrame(CameraRole::STEREO_RIGHT).cols == 100);
        if (drainedCount > 0) {
            assert(drainedFrame.timestamp > lastTimestamp);
        }
        lastTimestamp = drainedFrame.timestamp;
        drainedCount++;
    }
    assert(drainedCount <= 15);
    assert(drainedCount > 0);
    assert(lastTimestamp == 29);

    // 3. High-Concurrency Stress Test: 20,000 frames pushed by fast producer
    std::atomic<bool> producerDone{false};
    std::atomic<size_t> framesPopped{0};
    const size_t TOTAL_PRODUCE = 20000;

    std::thread producer([&]() {
        FrameSet prodFrame;
        prodFrame.preallocate(100, 100);
        for (size_t i = 1; i <= TOTAL_PRODUCE; ++i) {
            prodFrame.timestamp = i;
            ringBuffer.push(prodFrame);
        }
        producerDone = true;
    });

    std::thread consumer([&]() {
        FrameSet consFrame;
        consFrame.preallocate(100, 100);
        uint64_t prevTs = 0;
        while (!producerDone || !ringBuffer.empty()) {
            if (ringBuffer.pop(consFrame)) {
                framesPopped++;
                // Verify memory integrity: matrices must remain 100x100 and valid
                assert(!consFrame.getFrame(CameraRole::STEREO_LEFT).empty());
                assert(consFrame.getFrame(CameraRole::STEREO_LEFT).rows == 100);
                assert(consFrame.getFrame(CameraRole::STEREO_LEFT).cols == 100);
                assert(consFrame.timestamp > prevTs);
                prevTs = consFrame.timestamp;
            } else {
                std::this_thread::yield();
            }
        }
    });

    producer.join();
    consumer.join();

    assert(framesPopped > 0);
    spdlog::info("[TEST] AtomicRingBuffer overwrite verification passed (popped {}/{} frames under heavy contention).",
                 framesPopped.load(), TOTAL_PRODUCE);
}

void testAsyncFlightRecorderStream() {
    std::string testDir = "build/stream_test";
    std::filesystem::remove_all(testDir);

    FlightRecorder recorder(testDir);

    std::vector<RecordedFrame> streamFrames;
    for (int i = 0; i < 20; ++i) {
        RecordedFrame f;
        f.timestamp = 1000 + i;
        f.leftFrame = cv::Mat::zeros(80, 80, CV_8UC1);
        f.rightFrame = cv::Mat::zeros(80, 80, CV_8UC1);
        cv::circle(f.leftFrame, cv::Point(40, 40), 10, cv::Scalar(255), -1);
        cv::circle(f.rightFrame, cv::Point(40, 40), 10, cv::Scalar(255), -1);
        f.triggerDiag = {{"frame", i}};
        streamFrames.push_back(f);
    }

    auto start = std::chrono::steady_clock::now();
    recorder.saveStreamSession(streamFrames);
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - start);

    // saveStreamSession must be asynchronous: return immediately (< 10 ms)
    spdlog::info("[TEST] saveStreamSession non-blocking latency: {} us", duration.count());
    assert(duration.count() < 10000); // Less than 10 ms

    // Wait for background worker to complete writing files
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Verify stream folder and contents
    bool foundStreamDir = false;
    for (const auto& entry : std::filesystem::directory_iterator(testDir)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("stream_", 0) == 0) {
            foundStreamDir = true;
            assert(std::filesystem::exists(entry.path() / "raw"));
            assert(std::filesystem::exists(entry.path() / "annotated"));
            assert(std::filesystem::exists(entry.path() / "metadata.json"));

            // Check that left and right PNGs exist
            assert(std::filesystem::exists(entry.path() / "raw" / "left_000.png"));
            assert(std::filesystem::exists(entry.path() / "raw" / "right_000.png"));
        }
    }
    assert(foundStreamDir);

    std::filesystem::remove_all(testDir);
    spdlog::info("[TEST] Async FlightRecorder stream verification passed.");
}

// Mock components for testing SessionStateMachine in isolation
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

void testSessionStateMachineStroboscopicTiming() {
    // 1. Verify 3.0 ft default geometry in PipelineTimingConfig
    PipelineTimingConfig config;
    assert(std::abs(config.workingDistanceMeters - 0.9144) < 1e-4);
    assert(std::abs(config.pulseIntervalMs - 3.3333) < 1e-3);
    assert(config.minPointsToSolve == 3);
    assert(config.maxFramesPerShot == 2);
    assert(config.emptyFrameTimeout == 1);
    assert(std::abs(config.nominalBallRadiusPx - 23.3) < 0.1);
    assert(config.highStrobeRateHz == 300.0);
    assert(config.standbyStrobeRateHz == 10.0);
    assert(config.cameraExposureUs == 10000);
    assert(config.strobePulseCount == 3);
    assert(config.ballLossTimeoutSec == 5.0);
    assert(config.isValidTiming());

    // Helper to generate a dummy FrameSet
    auto makeFrameSet = []() {
        FrameSet fs;
        fs.preallocate(1280, 800);
        fs.timestamp = 1000;
        return fs;
    };

    // Helper to generate N Ball3D points
    auto makeBalls = [](int count) {
        std::vector<Ball3D> balls;
        for (int i = 0; i < count; ++i) {
            Ball3D b;
            b.centroid = Eigen::Vector3d(0.1 * i, 0.0, 0.9144);
            balls.push_back(b);
        }
        return balls;
    };

    // 2. Test Case 1: Low-Latency High-Speed Exit (Frame 1 has 5 pulses, Frame 2 is empty)
    {
        MockStrobeTrigger::fireOnNext = false;
        MockStrobeVision::returnObs.clear();
        MockStrobeSpatial::returnBalls.clear();
        MockStrobeKinematics::lastTrajectorySize = 0;
        MockStrobeKinematics::lastPulseIntervalMs = 0.0;
        MockStrobeNet::transmitted = false;

        MockStrobeTrigger trig;
        MockStrobeVision vis;
        MockStrobeSpatial spat;
        MockStrobeKinematics kin;
        MockStrobeNet net;

        SessionStateMachine<MockStrobeTrigger, MockStrobeVision, MockStrobeSpatial, MockStrobeKinematics, MockStrobeNet>
            ssm(trig, vis, spat, kin, net, config);

        // Frame 0: Trigger fires! Vision finds 5 pulses
        MockStrobeTrigger::fireOnNext = true;
        MockStrobeVision::returnObs.resize(5);
        MockStrobeSpatial::returnBalls = makeBalls(5);

        ssm.processNextFrame(makeFrameSet());
        assert(!MockStrobeNet::transmitted); // In-flight, waiting to confirm completion

        // Frame 1: Ball has exited FOV! Vision finds 0 pulses
        MockStrobeVision::returnObs.clear();
        MockStrobeSpatial::returnBalls.clear();

        ssm.processNextFrame(makeFrameSet());
        // With emptyFrameTimeout = 1, it must solve IMMEDIATELY without waiting 15 frames!
        assert(MockStrobeNet::transmitted);
        assert(MockStrobeKinematics::lastTrajectorySize == 5);
        assert(std::abs(MockStrobeKinematics::lastPulseIntervalMs - 3.3333) < 1e-3);
    }

    // 3. Test Case 2: 2-Frame Hybrid Accumulation for Irons (Frame 1 + Frame 2)
    {
        MockStrobeTrigger::fireOnNext = false;
        MockStrobeVision::returnObs.clear();
        MockStrobeSpatial::returnBalls.clear();
        MockStrobeKinematics::lastTrajectorySize = 0;
        MockStrobeKinematics::lastPulseIntervalMs = 0.0;
        MockStrobeNet::transmitted = false;

        MockStrobeTrigger trig;
        MockStrobeVision vis;
        MockStrobeSpatial spat;
        MockStrobeKinematics kin;
        MockStrobeNet net;

        SessionStateMachine<MockStrobeTrigger, MockStrobeVision, MockStrobeSpatial, MockStrobeKinematics, MockStrobeNet>
            ssm(trig, vis, spat, kin, net, config);

        // Frame 0: Trigger fires! 5 pulses captured in Frame 1
        MockStrobeTrigger::fireOnNext = true;
        MockStrobeVision::returnObs.resize(5);
        MockStrobeSpatial::returnBalls = makeBalls(5);
        ssm.processNextFrame(makeFrameSet());
        assert(!MockStrobeNet::transmitted);

        // Frame 1: Ball still in FOV! 5 more pulses captured in Frame 2
        ssm.processNextFrame(makeFrameSet());
        // Frame limit reached (shotFrameCount == maxFramesPerShot == 2) -> solves across 10 points!
        assert(MockStrobeNet::transmitted);
        assert(MockStrobeKinematics::lastTrajectorySize == 10);
        assert(std::abs(MockStrobeKinematics::lastPulseIntervalMs - 3.3333) < 1e-3);
    }

    // 4. Test Case 3: Custom Timing Configuration (pulseIntervalMs = 0.8, minPointsToSolve = 4)
    {
        MockStrobeTrigger::fireOnNext = false;
        MockStrobeVision::returnObs.clear();
        MockStrobeSpatial::returnBalls.clear();
        MockStrobeKinematics::lastTrajectorySize = 0;
        MockStrobeKinematics::lastPulseIntervalMs = 0.0;
        MockStrobeNet::transmitted = false;

        PipelineTimingConfig customConfig;
        customConfig.pulseIntervalMs = 0.8;
        customConfig.minPointsToSolve = 4;
        customConfig.maxFramesPerShot = 1;
        customConfig.emptyFrameTimeout = 1;

        MockStrobeTrigger trig;
        MockStrobeVision vis;
        MockStrobeSpatial spat;
        MockStrobeKinematics kin;
        MockStrobeNet net;

        SessionStateMachine<MockStrobeTrigger, MockStrobeVision, MockStrobeSpatial, MockStrobeKinematics, MockStrobeNet>
            ssm(trig, vis, spat, kin, net, customConfig);

        MockStrobeTrigger::fireOnNext = true;
        MockStrobeVision::returnObs.resize(4);
        MockStrobeSpatial::returnBalls = makeBalls(4);

        ssm.processNextFrame(makeFrameSet());
        // maxFramesPerShot == 1 reached immediately!
        assert(MockStrobeNet::transmitted);
        assert(MockStrobeKinematics::lastTrajectorySize == 4);
        assert(std::abs(MockStrobeKinematics::lastPulseIntervalMs - 0.8) < 1e-4);
    }

    spdlog::info("[TEST] SessionStateMachine stroboscopic timing and 3.0 ft geometry verification passed.");
}

void testSerialPort() {
    SerialPort serial;
    assert(!serial.isOpen());

    // Test retry connection failure on non-existent port (2 retries = 3 total attempts)
    // Use short delay (20 ms) to keep unit test runtime snappy (<100 ms total)
#ifdef _WIN32
    std::string fakePort = "COM99";
#else
    std::string fakePort = "/dev/tty_nonexistent_golfsim_test";
#endif
    bool success = serial.openWithRetry(fakePort, 115200, 2, 20);
    assert(!success);
    assert(!serial.isOpen());

    // Operations on unopened/failed port must fail gracefully without throwing or crashing
    assert(!serial.writeChar('H'));
    assert(!serial.writeString("TEST"));
    serial.flush();
    serial.close();

    spdlog::info("[TEST] SerialPort retry resilience and graceful degradation passed.");
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
    testAtomicRingBufferOverwrite();
    testAsyncFlightRecorderStream();
    testSessionStateMachineStroboscopicTiming();
    testSerialPort();

    spdlog::info("============================================");
    spdlog::info("All C++ Math Verification Tests PASSED!");
    spdlog::info("============================================");
}
