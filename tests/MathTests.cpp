#include "Math/Units.hpp"
#include "Math/BallPresenceTrigger.hpp"
#include "Math/StereoBallTrackerTrigger.hpp"
#include "Math/OpenCVMomentsTracker.hpp"
#include "Math/DotClusterFinder.hpp"
#include "Math/DotClusterTracker.hpp"
#include "Camera/CameraConfig.hpp"
#include "App/AppConfig.hpp"
#include "HAL/IUsbVideoDriver.hpp"
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
#include "TestAssert.hpp"
#include "TestRegistry.hpp"
#include "TestSandbox.hpp"
#include "Mocks.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <filesystem>
#include <thread>
#include <numbers>
#include <fstream>

// Synthetic dots-only ball: `dots` retroreflective glints spread over the lit
// cap of the ball (radius `capRadius`, ~0.45x the ball radius in practice).
// Dot 0 sits at the centre; the rest alternate between two rings.
static void drawDotBall(cv::Mat& frame, cv::Point centre, int capRadius = 10,
                        int dots = 9, int dotRadius = 2, int brightness = 235) {
    cv::circle(frame, centre, dotRadius, cv::Scalar(brightness), -1);
    for (int i = 1; i < dots; ++i) {
        const double angle = 2.0 * std::numbers::pi * i / (dots - 1);
        const double ring = (i % 2 == 0) ? capRadius : capRadius * 0.55;
        cv::Point p(centre.x + static_cast<int>(std::lround(ring * std::cos(angle))),
                    centre.y + static_cast<int>(std::lround(ring * std::sin(angle))));
        cv::circle(frame, p, dotRadius, cv::Scalar(brightness), -1);
    }
}

// Mock driver that records every control call, in order, for the config tests.
struct MockUsbVideoDriver : public IUsbVideoDriver {
    std::vector<std::string> calls;
    int exposureUs = -1, gain = -1, brightness = -1;
    bool autoExposure = true, autoGain = true;

    bool grabRawFrame(cv::Mat&) override { return false; }
    void setHardwareExposure(int us) override { calls.push_back("exposure"); exposureUs = CameraConfig::quantiseExposureUs(us); }
    void setHardwareGain(int g) override { calls.push_back("gain"); gain = g; }
    void setHardwareBrightness(int b) override { calls.push_back("brightness"); brightness = b; }
    void setAutoExposure(bool on) override { calls.push_back("autoExposure"); autoExposure = on; }
    void setAutoGain(bool on) override { calls.push_back("autoGain"); autoGain = on; }
    int getHardwareExposureUs() const override { return exposureUs; }
    int getHardwareGain() const override { return gain; }
    double getNegotiatedFps() const override { return 100.0; }
    void injectImmediateRegisterWrite(uint16_t, uint8_t) override {}
};


GOLFSIM_TEST(Units) {
    MetersPerSecond mps(10.0);
    MilesPerHour mph = to_mph(mps);
    TEST_ASSERT(std::abs(mph.value() - 22.36936) < 1e-4);

    Radians rad(std::numbers::pi / 4.0);
    Degrees deg = to_degrees(rad);
    TEST_ASSERT(std::abs(deg.value() - 45.0) < 1e-4);

    Degrees deg2(90.0);
    Radians rad2 = to_radians(deg2);
    TEST_ASSERT(std::abs(rad2.value() - std::numbers::pi / 2.0) < 1e-6);

    MetersPerSecond mps2 = to_mps(mph);
    TEST_ASSERT(std::abs(mps2.value() - 10.0) < 1e-5);

    spdlog::info("[TEST] Units verification passed.");
}

GOLFSIM_TEST(BallPresenceTrigger) {
    // Dot-cluster finder, 5-frame lock, whole-frame search (no tee ROI)
    DotClusterConfig dots;
    dots.intensityThreshold = 100;
    BallPresenceTrigger trigger(dots, 23.3, 5, 0.5, 5.0);

    // Frame with a stable dots-only ball anywhere in view -- here well away
    // from where the old tee ROI used to be.
    cv::Mat ballFrame = cv::Mat::zeros(200, 300, CV_8UC1);
    drawDotBall(ballFrame, cv::Point(240, 40));

    // Feed for 4 frames (under stability threshold 5)
    for (int i = 0; i < 4; ++i) {
        bool trig = trigger.checkOpticalGate(ballFrame);
        TEST_ASSERT(!trig);
        nlohmann::json diag = trigger.getLatestDiagnostics();
        TEST_ASSERT(diag["state"] == "WAITING_FOR_BALL");
        TEST_ASSERT(diag["stabilityCounter"] == i + 1);
        TEST_ASSERT(diag["candidateCount"] == 1);
        TEST_ASSERT(!diag.contains("teeROI"));
    }

    // 5th frame reaches stability threshold -> BALL_LOCKED
    bool trigLock = trigger.checkOpticalGate(ballFrame);
    TEST_ASSERT(!trigLock);
    nlohmann::json diagLocked = trigger.getLatestDiagnostics();
    TEST_ASSERT(diagLocked["state"] == "BALL_LOCKED");
    auto lbox = diagLocked["lockedBallBox"];
    TEST_ASSERT(lbox[2].get<int>() > 0);
    // Locked box is the nominal ball box around the cluster centroid
    TEST_NEAR(lbox[0].get<int>() + lbox[2].get<int>() / 2.0, 240.0, 3.0);
    TEST_NEAR(lbox[1].get<int>() + lbox[3].get<int>() / 2.0, 40.0, 3.0);

    // Shadow Test: Dim the dots by 30% (hand/club shadow or IR fluctuation)
    cv::Mat dimmedFrame = cv::Mat::zeros(200, 300, CV_8UC1);
    drawDotBall(dimmedFrame, cv::Point(240, 40), 10, 9, 2, 165);
    bool trigShadow = trigger.checkOpticalGate(dimmedFrame);
    TEST_ASSERT(!trigShadow); // SHADOW IMMUNITY: must NOT trigger false departure
    nlohmann::json diagShadow = trigger.getLatestDiagnostics();
    TEST_ASSERT(diagShadow["state"] == "BALL_LOCKED");
    TEST_ASSERT(diagShadow["matchScore"].get<float>() > 0.70f);

    // Departure Test: Feed black frame (ball physically departed from tee)
    cv::Mat blankFrame = cv::Mat::zeros(200, 300, CV_8UC1);
    bool trigDeparted = trigger.checkOpticalGate(blankFrame);
    TEST_ASSERT(trigDeparted); // Triggered! Dot pattern vanished.
    nlohmann::json diagDeparted = trigger.getLatestDiagnostics();
    TEST_ASSERT(diagDeparted["state"] == "BALL_DEPARTED");

    // Test reset
    trigger.reset();
    nlohmann::json diagReset = trigger.getLatestDiagnostics();
    TEST_ASSERT(diagReset["state"] == "WAITING_FOR_BALL");
    TEST_ASSERT(diagReset["stabilityCounter"] == 0);
    TEST_ASSERT(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);

    // A lone reflection is not a ball: no lock progress
    cv::Mat sparkleFrame = cv::Mat::zeros(200, 300, CV_8UC1);
    cv::circle(sparkleFrame, cv::Point(100, 100), 2, cv::Scalar(240), -1);
    trigger.checkOpticalGate(sparkleFrame);
    TEST_ASSERT(trigger.getLatestDiagnostics()["candidateCount"] == 0);
    TEST_ASSERT(trigger.getLatestDiagnostics()["stabilityCounter"] == 0);

    // Emitter Protection Test: Fast loss timeout (50ms)
    trigger.setLossTimeoutSec(0.05); // 50 ms test timeout
    trigger.checkOpticalGate(blankFrame); // Starts empty timer
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    trigger.checkOpticalGate(blankFrame); // Triggers timeout
    TEST_ASSERT(trigger.getEmitterMode() == EmitterPowerMode::LOW_STANDBY);
    TEST_ASSERT(trigger.isStandbyRequested());
    nlohmann::json diagStandby = trigger.getLatestDiagnostics();
    TEST_ASSERT(diagStandby["emitterMode"] == "STANDBY");

    // Restoring ball on tee immediately wakes up to HIGH_STROBE_READY
    trigger.checkOpticalGate(ballFrame);
    TEST_ASSERT(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);
    TEST_ASSERT(!trigger.isStandbyRequested());

    // Verify ITriggerDetector interface polymorphism and zero recursion
    ITriggerDetector* baseDetector = &trigger;
    bool trigPoly1 = baseDetector->checkTrigger(ballFrame, ballFrame);
    bool trigPoly2 = baseDetector->checkOpticalGate(ballFrame);
    (void)trigPoly1;
    (void)trigPoly2;

    spdlog::info("[TEST] BallPresenceTrigger verification passed (dot clusters, whole frame, emitter protection).");
}

GOLFSIM_TEST(OpenCVMomentsTracker) {
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
    TEST_ASSERT(balls.size() == 1);
    TEST_ASSERT(std::abs(balls[0].centroid.x - 200) < 1.0);
    TEST_ASSERT(std::abs(balls[0].centroid.y - 200) < 1.0);
    TEST_ASSERT(balls[0].markers.size() == 1);
    TEST_ASSERT(std::abs(balls[0].markers[0].position.x - 205) < 1.0);
    TEST_ASSERT(std::abs(balls[0].markers[0].position.y - 195) < 1.0);

    // Verify diagnostics
    nlohmann::json vdiag = tracker.getLatestDiagnostics();
    TEST_ASSERT(vdiag["candidates"].size() >= 3);
    bool foundBall = false;
    bool foundNoiseArea = false;
    bool foundNoiseCirc = false;
    for (const auto& cand : vdiag["candidates"]) {
        bool accepted = cand.value("accepted", false);
        std::string reason = cand.value("reason", "");
        if (accepted) {
            auto cen = cand["centroid"];
            TEST_ASSERT(std::abs(cen[0].get<double>() - 200) < 1.0);
            TEST_ASSERT(reason == "Accepted (Moments)");
            TEST_ASSERT(cand["markers"].size() == 1);
            TEST_ASSERT(std::abs(cand["markers"][0][0].get<double>() - 205) < 1.0);
            foundBall = true;
        } else if (reason == "Area too small") {
            foundNoiseArea = true;
        } else if (reason == "Circularity too low") {
            foundNoiseCirc = true;
        }
    }
    TEST_ASSERT(foundBall);
    TEST_ASSERT(foundNoiseArea);
    TEST_ASSERT(foundNoiseCirc);

    spdlog::info("[TEST] OpenCVMomentsTracker verification passed.");
}

GOLFSIM_TEST(StereoTriangulatorAndRaySphere) {
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
    TEST_ASSERT(result3D.size() == 1);
    TEST_ASSERT(std::abs(result3D[0].centroid.z() - 1.5) < 1e-3);
    TEST_ASSERT(std::abs(result3D[0].centroid.x() - 0.0) < 1e-3);
    TEST_ASSERT(std::abs(result3D[0].centroid.y() - 0.0) < 1e-3);

    TEST_ASSERT(result3D[0].markers.size() == 1);
    TEST_ASSERT(result3D[0].markers[0].isStereo);
    TEST_ASSERT(std::abs(result3D[0].markers[0].position.y() - 0.021335) < 1e-3);

    // Let's test the single-camera ray-sphere fallback.
    // If the marker is only visible in Left camera, we remove it from Right:
    bR.markers.clear();
    auto resultSingle = solver.triangulateShot({ bL }, { bR });
    TEST_ASSERT(resultSingle.size() == 1);
    TEST_ASSERT(resultSingle[0].markers.size() == 1);
    TEST_ASSERT(!resultSingle[0].markers[0].isStereo);
    TEST_ASSERT(resultSingle[0].markers[0].confidence == 0.5);
    // The recovered position should be extremely close to (0.0, 0.021335, 1.5)
    TEST_ASSERT(std::abs(resultSingle[0].markers[0].position.y() - 0.021335) < 1e-3);
    TEST_ASSERT(std::abs(resultSingle[0].markers[0].position.z() - 1.5) < 1e-3);

    // Test single-camera ray-sphere fallback for Right camera:
    // If marker is only visible in Right camera, remove it from Left:
    bL.markers.clear();
    bR.markers.push_back(mr);
    auto resultRightOnly = solver.triangulateShot({ bL }, { bR });
    TEST_ASSERT(resultRightOnly.size() == 1);
    TEST_ASSERT(resultRightOnly[0].markers.size() == 1);
    TEST_ASSERT(!resultRightOnly[0].markers[0].isStereo);
    TEST_ASSERT(resultRightOnly[0].markers[0].confidence == 0.5);
    // The recovered position in world (Left camera) coords must match (0.0, 0.021335, 1.5)
    TEST_ASSERT(std::abs(resultRightOnly[0].markers[0].position.x() - 0.0) < 1e-3);
    TEST_ASSERT(std::abs(resultRightOnly[0].markers[0].position.y() - 0.021335) < 1e-3);
    TEST_ASSERT(std::abs(resultRightOnly[0].markers[0].position.z() - 1.5) < 1e-3);

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
    TEST_ASSERT(steepTrajectory.size() == 3);
    // Principal motion vector projection must restore exact chronological order: p0 -> p1 -> p2
    // In world coordinates, +Y is down, so p0 (lowest) has highest Y, p2 (highest) has lowest Y.
    TEST_ASSERT(steepTrajectory[0].centroid.y() > steepTrajectory[1].centroid.y());
    TEST_ASSERT(steepTrajectory[1].centroid.y() > steepTrajectory[2].centroid.y());

    spdlog::info("[TEST] StereoTriangulator and Ray-Sphere Fallback verification passed.");
}

GOLFSIM_TEST(KinematicsEngine) {
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
    TEST_ASSERT(std::abs(data.ballSpeed.value() - expected_speed_mph) < 1e-2);

    // Verify launch angles using the corrected physical equations:
    // VLA = atan2(-vy, sqrt(vx^2 + vz^2))
    // HLA = atan2(vz, vx)
    double expected_vla = std::atan2(-velocity.y(), std::sqrt(velocity.x()*velocity.x() + velocity.z()*velocity.z())) * 180.0 / std::numbers::pi;
    double expected_hla = std::atan2(velocity.z(), velocity.x()) * 180.0 / std::numbers::pi;
    TEST_ASSERT(std::abs(data.verticalLaunchAngle.value() - expected_vla) < 1e-2);
    TEST_ASSERT(std::abs(data.horizontalLaunchAngle.value() - expected_hla) < 1e-2);

    // Verify spin speed and axis (solver negates Y component of spin axis for world coords)
    TEST_ASSERT(std::abs(data.spinRPM - 3000.0) < 1.0);
    Eigen::Vector3d expected_world_spin_axis(axis.x(), -axis.y(), axis.z());
    TEST_ASSERT((data.spinAxis - expected_world_spin_axis.normalized()).norm() < 1e-3);

    spdlog::info("[TEST] EigenBallisticsEngine verification passed.");
}

GOLFSIM_TEST(FlightRecorder) {
    const std::string testDir = TestSandbox::path("replays_test");
    std::filesystem::remove_all(testDir);

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

    // Call saveSession 12 times to trigger limit rotation (which caps at 10).
    // The recorder is scoped so its destructor drains the async queue before
    // we inspect the directory — deterministic, no sleep needed.
    {
        FlightRecorder recorder(testDir);
        for (int i = 0; i < 12; ++i) {
            recorder.saveSession(frames, launchData);
            // sleep a tiny bit to ensure distinct ms timestamps in folder names
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // Check that we have exactly 10 directories starting with shot_
    int count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(testDir)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("shot_", 0) == 0) {
            count++;
            // Check that subdirectories raw/ and annotated/ and metadata.json exist
            TEST_ASSERT(std::filesystem::exists(entry.path() / "raw"));
            TEST_ASSERT(std::filesystem::exists(entry.path() / "annotated"));
            TEST_ASSERT(std::filesystem::exists(entry.path() / "metadata.json"));
        }
    }
    TEST_ASSERT(count == 10);

    // Clean up
    std::filesystem::remove_all(testDir);
    spdlog::info("[TEST] FlightRecorder verification passed.");
}

GOLFSIM_TEST(StereoBallTrackerTrigger) {
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

    // Dot-cluster finder, whole-frame search, 5-frame stability lock, 3ft (0.9144m) limit.
    // Epipolar tolerance 30 px, disparity 5..600 px (AppConfig.stereo defaults but tighter epipolar).
    DotClusterConfig dots;
    dots.intensityThreshold = 100;
    StereoBallTrackerTrigger trigger(calib, dots, 23.3, 30.0, 5.0, 600.0, 256, 0.9144, 5, 4, 4.0, 0.04);

    // Frame setup for ball at (0, 0, 0.6) m (2 ft distance)
    // f = 1000, B = 0.1m => Disparity d = (f * B) / Z = 100 / 0.6 = 166.67 px
    // Left center (640, 400), Right center (640 - 167 = 473, 400)
    // Lit cap at 0.6m ~ 0.45 x 35.56 px ball radius -> cap radius 16 px
    cv::Mat frameL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat frameR = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(frameL, cv::Point(640, 400), 16);
    drawDotBall(frameR, cv::Point(473, 400), 16);

    // 0. Test 3FT DISTANCE REJECTION (Ball placed at Z = 1.0m / 3.28 ft > 3.0 ft)
    // f = 1000, B = 0.1m => d = 100/1.0 = 100 px => Right center = 640 - 100 = 540
    cv::Mat farFrameL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat farFrameR = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(farFrameL, cv::Point(640, 400), 10);
    drawDotBall(farFrameR, cv::Point(540, 400), 10);

    for (int i = 0; i < 6; ++i) {
        bool trigFar = trigger.checkTrigger(farFrameL, farFrameR);
        TEST_ASSERT(!trigFar);
        TEST_ASSERT(trigger.getState() == StereoTriggerState::SEARCHING);
    }

    // 1. Test SEARCHING -> ARMED Transition (Requires 5 consecutive stable frames)
    for (int i = 0; i < 4; ++i) {
        bool trigSearch = trigger.checkTrigger(frameL, frameR);
        TEST_ASSERT(!trigSearch);
        TEST_ASSERT(trigger.getState() == StereoTriggerState::SEARCHING);
    }
    // 5th frame transitions state to ARMED
    bool trigArmed = trigger.checkTrigger(frameL, frameR);
    TEST_ASSERT(!trigArmed);
    TEST_ASSERT(trigger.getState() == StereoTriggerState::ARMED);
    auto pos3D = trigger.getLastKnown3DPosition();
    TEST_ASSERT(std::abs(pos3D.z() - 0.6) < 0.02);

    // 2. Test SINGLE-CAMERA OCCLUSION IMMUNITY (hand behind ball): the left
    // camera still sees the dots, the right camera sees nothing
    cv::Mat occludedFrameL = frameL.clone();
    cv::Mat blankFrameR = cv::Mat::zeros(800, 1280, CV_8UC1);

    bool trigOcc = trigger.checkTrigger(occludedFrameL, blankFrameR);
    TEST_ASSERT(!trigOcc);
    TEST_ASSERT(trigger.getState() == StereoTriggerState::ARMED);

    // 3. Test DUAL-CAMERA LOSS GRACE WINDOW: graceMax = 4 frames of loss are
    // tolerated (the occlusion frame above did not count -- one camera still saw
    // the ball), the 5th drops back to SEARCHING.
    for (int i = 0; i < 4; ++i) {
        bool trigGrace = trigger.checkTrigger(blankFrameR, blankFrameR);
        TEST_ASSERT(!trigGrace);
        TEST_ASSERT(trigger.getState() == StereoTriggerState::ARMED);
    }
    bool trigReset = trigger.checkTrigger(blankFrameR, blankFrameR);
    TEST_ASSERT(!trigReset);
    TEST_ASSERT(trigger.getState() == StereoTriggerState::SEARCHING);

    // Re-lock ball to ARMED (requires 5 frames)
    for (int i = 0; i < 5; ++i) {
        trigger.checkTrigger(frameL, frameR);
    }
    TEST_ASSERT(trigger.getState() == StereoTriggerState::ARMED);

    // 4. Test VIBRATION / NUDGE REJECTION
    // Displace ball slightly by 45 mm (beyond 40 mm threshold), but low speed (0.5 m/s)
    // At Z=0.6m, 45mm horizontal displacement = ~75 px -> u_L = 640 + 75 = 715, u_R = 473 + 75 = 548
    cv::Mat nudgeL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat nudgeR = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(nudgeL, cv::Point(715, 400), 16);
    drawDotBall(nudgeR, cv::Point(548, 400), 16);

    bool trigNudge = trigger.checkTrigger(nudgeL, nudgeR);
    TEST_ASSERT(!trigNudge);
    TEST_ASSERT(trigger.getState() == StereoTriggerState::CONFIRMING);

    for (int i = 0; i < 3; ++i) {
        bool trigConfirm = trigger.checkTrigger(nudgeL, nudgeR);
        TEST_ASSERT(!trigConfirm);
    }
    TEST_ASSERT(trigger.getState() == StereoTriggerState::ARMED);

    // 5. Test VALID HIGH-VELOCITY IMPACT TRIGGER
    // Displace ball at launch speed 40 m/s (~90 mph)
    cv::Mat launchL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat launchR = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(launchL, cv::Point(800, 400), 16);
    drawDotBall(launchR, cv::Point(633, 400), 16);

    bool trigImpact1 = trigger.checkTrigger(launchL, launchR);
    TEST_ASSERT(!trigImpact1);
    TEST_ASSERT(trigger.getState() == StereoTriggerState::CONFIRMING);

    cv::Mat launchL2 = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat launchR2 = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(launchL2, cv::Point(900, 400), 16);
    drawDotBall(launchR2, cv::Point(733, 400), 16);

    bool trigImpact2 = trigger.checkTrigger(launchL2, launchR2);
    TEST_ASSERT(trigImpact2);
    TEST_ASSERT(trigger.getState() == StereoTriggerState::CAPTURED);

    // 6. Test STANDBY EMITTER PROTECTION (Dynamic Photobiological Safety)
    trigger.reset();
    TEST_ASSERT(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);
    TEST_ASSERT(!trigger.isStandbyRequested());

    // Set fast test timeout (50ms)
    trigger.setLossTimeoutSec(0.05);
    trigger.checkTrigger(blankFrameR, blankFrameR); // Starts empty timer
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    trigger.checkTrigger(blankFrameR, blankFrameR); // Triggers transition to LOW_STANDBY
    TEST_ASSERT(trigger.getEmitterMode() == EmitterPowerMode::LOW_STANDBY);
    TEST_ASSERT(trigger.isStandbyRequested());

    // Placing ball back on tee restores HIGH_STROBE_READY
    trigger.checkTrigger(frameL, frameR);
    TEST_ASSERT(trigger.getEmitterMode() == EmitterPowerMode::HIGH_STROBE_READY);
    TEST_ASSERT(!trigger.isStandbyRequested());

    // 7. WHOLE-FRAME SEARCH: ball in a corner (outside any old ROI) next to a
    // bright specular bar (19x59 px, like the one measured on the rig) still locks.
    trigger.reset();
    cv::Mat cornerL = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::Mat cornerR = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(cornerL, cv::Point(200, 120), 16);
    drawDotBall(cornerR, cv::Point(33, 120), 16);
    cv::rectangle(cornerL, cv::Rect(230, 20, 19, 59), cv::Scalar(255), -1);
    cv::rectangle(cornerR, cv::Rect(63, 20, 19, 59), cv::Scalar(255), -1);
    for (int i = 0; i < 5; ++i) {
        trigger.checkTrigger(cornerL, cornerR);
    }
    TEST_ASSERT(trigger.getState() == StereoTriggerState::ARMED);
    TEST_NEAR(trigger.getLastKnown3DPosition().z(), 0.6, 0.02);
    // The locking frame's diagnostics: one dot cluster per camera (the bar was
    // rejected), and the stereo offsets a stream recording would report.
    nlohmann::json cornerDiag = trigger.getLatestDiagnostics();
    TEST_ASSERT(cornerDiag["leftCandidates"] == 1);
    TEST_ASSERT(cornerDiag["rightCandidates"] == 1);
    TEST_NEAR(cornerDiag["disparityPx"].get<double>(), 167.0, 1.5);
    TEST_NEAR(cornerDiag["verticalOffsetPx"].get<double>(), 0.0, 1.0);
    TEST_ASSERT(cornerDiag["leftDots"]["candidates"].size() == 1);
    TEST_ASSERT(cornerDiag["leftDots"]["rejectedDotCount"] == 1);

    spdlog::info("[TEST] StereoBallTrackerTrigger verification passed (dot clusters, whole frame, emitter protection).");
}

GOLFSIM_TEST(AtomicRingBufferOverwrite) {
    AtomicRingBuffer<FrameSet, 16> ringBuffer;
    ringBuffer.preallocate(100, 100);

    TEST_ASSERT(ringBuffer.empty());
    TEST_ASSERT(ringBuffer.size() == 0);
    TEST_ASSERT(ringBuffer.capacity() == 15);

    // 1. Test basic push and pop
    FrameSet pushFrame;
    pushFrame.preallocate(100, 100);
    pushFrame.timestamp = 1001;

    ringBuffer.push(pushFrame);
    TEST_ASSERT(!ringBuffer.empty());
    TEST_ASSERT(ringBuffer.size() == 1);

    FrameSet popFrame;
    popFrame.preallocate(100, 100);
    bool popSuccess = ringBuffer.pop(popFrame);
    TEST_ASSERT(popSuccess);
    TEST_ASSERT(popFrame.timestamp == 1001);
    TEST_ASSERT(popFrame.getFrame(CameraRole::STEREO_LEFT).rows == 100);
    TEST_ASSERT(popFrame.getFrame(CameraRole::STEREO_LEFT).cols == 100);
    TEST_ASSERT(ringBuffer.empty());

    // 2. Test overwrite semantics (pushing 30 items into capacity 15 buffer)
    for (uint64_t i = 0; i < 30; ++i) {
        FrameSet f;
        f.preallocate(100, 100);
        f.timestamp = i;
        ringBuffer.push(f);
    }

    // Since capacity is 15, size should be <= 15
    TEST_ASSERT(ringBuffer.size() <= 15);

    // Drain and verify monotonic increasing timestamps and no corrupted frames
    uint64_t lastTimestamp = 0;
    size_t drainedCount = 0;
    FrameSet drainedFrame;
    drainedFrame.preallocate(100, 100);
    while (ringBuffer.pop(drainedFrame)) {
        TEST_ASSERT(drainedFrame.getFrame(CameraRole::STEREO_LEFT).rows == 100);
        TEST_ASSERT(drainedFrame.getFrame(CameraRole::STEREO_RIGHT).cols == 100);
        if (drainedCount > 0) {
            TEST_ASSERT(drainedFrame.timestamp > lastTimestamp);
        }
        lastTimestamp = drainedFrame.timestamp;
        drainedCount++;
    }
    TEST_ASSERT(drainedCount <= 15);
    TEST_ASSERT(drainedCount > 0);
    TEST_ASSERT(lastTimestamp == 29);

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
                TEST_ASSERT(!consFrame.getFrame(CameraRole::STEREO_LEFT).empty());
                TEST_ASSERT(consFrame.getFrame(CameraRole::STEREO_LEFT).rows == 100);
                TEST_ASSERT(consFrame.getFrame(CameraRole::STEREO_LEFT).cols == 100);
                TEST_ASSERT(consFrame.timestamp > prevTs);
                prevTs = consFrame.timestamp;
            } else {
                std::this_thread::yield();
            }
        }
    });

    producer.join();
    consumer.join();

    TEST_ASSERT(framesPopped > 0);
    spdlog::info("[TEST] AtomicRingBuffer overwrite verification passed (popped {}/{} frames under heavy contention).",
                 framesPopped.load(), TOTAL_PRODUCE);
}

GOLFSIM_TEST(AsyncFlightRecorderStream) {
    const std::string testDir = TestSandbox::path("stream_test");
    std::filesystem::remove_all(testDir);

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

    std::chrono::microseconds duration{};
    {
        FlightRecorder recorder(testDir);
        auto start = std::chrono::steady_clock::now();
        recorder.saveStreamSession(streamFrames);
        duration = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start);
    }   // destructor drains the worker queue: files are on disk here

    // saveStreamSession must be asynchronous: return immediately (< 10 ms)
    spdlog::info("[TEST] saveStreamSession non-blocking latency: {} us", duration.count());
    TEST_ASSERT(duration.count() < 10000); // Less than 10 ms

    // Verify stream folder and contents
    bool foundStreamDir = false;
    for (const auto& entry : std::filesystem::directory_iterator(testDir)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("stream_", 0) == 0) {
            foundStreamDir = true;
            TEST_ASSERT(std::filesystem::exists(entry.path() / "raw"));
            TEST_ASSERT(std::filesystem::exists(entry.path() / "annotated"));
            TEST_ASSERT(std::filesystem::exists(entry.path() / "metadata.json"));

            // Check that left and right PNGs exist
            TEST_ASSERT(std::filesystem::exists(entry.path() / "raw" / "left_000.png"));
            TEST_ASSERT(std::filesystem::exists(entry.path() / "raw" / "right_000.png"));
        }
    }
    TEST_ASSERT(foundStreamDir);

    std::filesystem::remove_all(testDir);
    spdlog::info("[TEST] Async FlightRecorder stream verification passed.");
}

GOLFSIM_TEST(SessionStateMachineStroboscopicTiming) {
    // Solved mock shots are recorded under the sandbox, never build/replays
    const std::string ssmReplayDir   = TestSandbox::path("ssm_replays");
    const std::string ssmHistoryPath = TestSandbox::path("ssm_shot_history.json");
    std::filesystem::remove_all(ssmReplayDir);
    std::filesystem::remove(ssmHistoryPath);

    // 1. Verify 3.0 ft default geometry in PipelineTimingConfig
    PipelineTimingConfig config;
    TEST_ASSERT(std::abs(config.workingDistanceMeters - 0.9144) < 1e-4);
    TEST_ASSERT(std::abs(config.pulseIntervalMs - 3.3333) < 1e-3);
    TEST_ASSERT(config.minPointsToSolve == 3);
    TEST_ASSERT(config.maxFramesPerShot == 2);
    TEST_ASSERT(config.emptyFrameTimeout == 1);
    TEST_ASSERT(std::abs(config.nominalBallRadiusPx - 23.3) < 0.1);
    TEST_ASSERT(config.highStrobeRateHz == 300.0);
    TEST_ASSERT(config.standbyStrobeRateHz == 10.0);
    TEST_ASSERT(config.cameraExposureUs == 7812);
    TEST_ASSERT(config.cameraFrameRateHz == 100.0);
    TEST_ASSERT(config.strobePulseCount == 3);
    TEST_ASSERT(config.ballLossTimeoutSec == 5.0);
    TEST_ASSERT(config.isValidTiming());
    TEST_NEAR(config.strobeTrainDurationUs(), 6696.6, 1.0);

    // Exposure must hold the full pulse train and fit inside one frame period
    {
        PipelineTimingConfig t = config;
        t.cameraExposureUs = 2000;          // 1 pulse only
        TEST_ASSERT(!t.isValidTiming());
        t.cameraExposureUs = 3906;          // 2 pulses
        TEST_ASSERT(!t.isValidTiming());
        t.strobePulseCount = 2;
        TEST_ASSERT(t.isValidTiming());
        t.strobePulseCount = 3;
        t.cameraExposureUs = 7812;
        TEST_ASSERT(t.isValidTiming());
        t.cameraExposureUs = 15625;         // longer than the 10 ms frame period
        TEST_ASSERT(!t.isValidTiming());
        t.cameraFrameRateHz = 30.0;         // ...but fine at 30 fps
        TEST_ASSERT(t.isValidTiming());
    }

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
            ssm(trig, vis, spat, kin, net, config, ssmReplayDir, ssmHistoryPath);

        // Frame 0: Trigger fires! Vision finds 5 pulses
        MockStrobeTrigger::fireOnNext = true;
        MockStrobeVision::returnObs.resize(5);
        MockStrobeSpatial::returnBalls = makeBalls(5);

        ssm.processNextFrame(makeFrameSet());
        TEST_ASSERT(!MockStrobeNet::transmitted); // In-flight, waiting to confirm completion

        // Frame 1: Ball has exited FOV! Vision finds 0 pulses
        MockStrobeVision::returnObs.clear();
        MockStrobeSpatial::returnBalls.clear();

        ssm.processNextFrame(makeFrameSet());
        // With emptyFrameTimeout = 1, it must solve IMMEDIATELY without waiting 15 frames!
        TEST_ASSERT(MockStrobeNet::transmitted);
        TEST_ASSERT(MockStrobeKinematics::lastTrajectorySize == 5);
        TEST_ASSERT(std::abs(MockStrobeKinematics::lastPulseIntervalMs - 3.3333) < 1e-3);
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
            ssm(trig, vis, spat, kin, net, config, ssmReplayDir, ssmHistoryPath);

        // Frame 0: Trigger fires! 5 pulses captured in Frame 1
        MockStrobeTrigger::fireOnNext = true;
        MockStrobeVision::returnObs.resize(5);
        MockStrobeSpatial::returnBalls = makeBalls(5);
        ssm.processNextFrame(makeFrameSet());
        TEST_ASSERT(!MockStrobeNet::transmitted);

        // Frame 1: Ball still in FOV! 5 more pulses captured in Frame 2
        ssm.processNextFrame(makeFrameSet());
        // Frame limit reached (shotFrameCount == maxFramesPerShot == 2) -> solves across 10 points!
        TEST_ASSERT(MockStrobeNet::transmitted);
        TEST_ASSERT(MockStrobeKinematics::lastTrajectorySize == 10);
        TEST_ASSERT(std::abs(MockStrobeKinematics::lastPulseIntervalMs - 3.3333) < 1e-3);
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
            ssm(trig, vis, spat, kin, net, customConfig, ssmReplayDir, ssmHistoryPath);

        MockStrobeTrigger::fireOnNext = true;
        MockStrobeVision::returnObs.resize(4);
        MockStrobeSpatial::returnBalls = makeBalls(4);

        ssm.processNextFrame(makeFrameSet());
        // maxFramesPerShot == 1 reached immediately!
        TEST_ASSERT(MockStrobeNet::transmitted);
        TEST_ASSERT(MockStrobeKinematics::lastTrajectorySize == 4);
        TEST_ASSERT(std::abs(MockStrobeKinematics::lastPulseIntervalMs - 0.8) < 1e-4);
    }

    spdlog::info("[TEST] SessionStateMachine stroboscopic timing and 3.0 ft geometry verification passed.");
}

GOLFSIM_TEST(SerialPort) {
    SerialPort serial;
    TEST_ASSERT(!serial.isOpen());

    // Test retry connection failure on non-existent port (2 retries = 3 total attempts)
    // Use short delay (20 ms) to keep unit test runtime snappy (<100 ms total)
#ifdef _WIN32
    std::string fakePort = "COM99";
#else
    std::string fakePort = "/dev/tty_nonexistent_golfsim_test";
#endif
    bool success = serial.openWithRetry(fakePort, 115200, 2, 20);
    TEST_ASSERT(!success);
    TEST_ASSERT(!serial.isOpen());

    // Operations on unopened/failed port must fail gracefully without throwing or crashing
    TEST_ASSERT(!serial.writeChar('H'));
    TEST_ASSERT(!serial.writeString("TEST"));
    serial.flush();
    serial.close();

    spdlog::info("[TEST] SerialPort retry resilience and graceful degradation passed.");
}

GOLFSIM_TEST(CameraConfig) {
    // UVC exposure is log2(seconds): round to the NEAREST step, never floor
    TEST_ASSERT(CameraConfig::exposureUsToLog2(2000)  == -9);
    TEST_ASSERT(CameraConfig::quantiseExposureUs(2000)  == 1953);
    TEST_ASSERT(CameraConfig::quantiseExposureUs(5000)  == 3906);
    TEST_ASSERT(CameraConfig::quantiseExposureUs(10000) == 7812);
    TEST_ASSERT(CameraConfig::quantiseExposureUs(3000)  == 3906);   // floor() would give 1953
    TEST_ASSERT(CameraConfig::quantiseExposureUs(7812)  == 7812);
    TEST_ASSERT(CameraConfig::quantiseExposureUs(50)    == 122);    // clamped to the minimum step
    TEST_ASSERT(CameraConfig::quantiseExposureUs(0)     == 122);
    TEST_ASSERT(CameraConfig::exposureLog2ToUs(-7)  == 7812);
    TEST_ASSERT(CameraConfig::exposureLog2ToUs(-13) == 122);

    // Defaults hold the 3-pulse 300 Hz train and fit the 100 fps frame period
    CameraConfig def;
    TEST_ASSERT(def.exposureUs == 7812);
    TEST_ASSERT(def.gain == 0);
    TEST_ASSERT(def.brightness == 0);
    TEST_ASSERT(def.targetFps == 100);
    TEST_ASSERT(!def.autoExposure);
    TEST_ASSERT(def.framePeriodUs() == 10000);
    TEST_ASSERT(def.exposureUs < def.framePeriodUs());   // 7812 us fits the 100 fps frame period

    // JSON round trip
    CameraConfig a;
    a.exposureUs = 3906; a.gain = 42; a.brightness = 3; a.targetFps = 60;
    CameraConfig b = CameraConfig::fromJson(a.toJson());
    TEST_ASSERT(b.exposureUs == 3906 && b.gain == 42 && b.brightness == 3 && b.targetFps == 60);

    // Partial JSON only overrides what it names
    CameraConfig c = CameraConfig::fromJson(nlohmann::json::parse(R"({"gain": 25})"), a);
    TEST_ASSERT(c.gain == 25 && c.exposureUs == 3906 && c.targetFps == 60);

    // Out-of-range values clamp to the hardware ranges
    CameraConfig d = CameraConfig::fromJson(nlohmann::json::parse(R"({"gain": 500, "brightness": -4})"));
    TEST_ASSERT(d.gain == CameraConfig::kMaxGain);
    TEST_ASSERT(d.brightness == 0);

    // applyCameraConfig issues the controls in UVC order: auto off, exposure, gain, brightness
    MockUsbVideoDriver mock;
    mock.applyCameraConfig(a);
    TEST_ASSERT(mock.calls.size() == 5);
    TEST_ASSERT(mock.calls[0] == "autoExposure");
    TEST_ASSERT(mock.calls[1] == "autoGain");
    TEST_ASSERT(mock.calls[2] == "exposure");
    TEST_ASSERT(mock.calls[3] == "gain");
    TEST_ASSERT(mock.calls[4] == "brightness");
    TEST_ASSERT(!mock.autoExposure);
    TEST_ASSERT(mock.getHardwareExposureUs() == 3906);
    TEST_ASSERT(mock.getHardwareGain() == 42);
    TEST_ASSERT(mock.brightness == 3);

    spdlog::info("[TEST] CameraConfig quantisation, JSON and control ordering passed.");
}

GOLFSIM_TEST(AppConfig) {
    // Compiled defaults
    AppConfig def;
    TEST_ASSERT(def.camera.exposureUs == 7812);
    TEST_ASSERT(def.detector.intensityThreshold == 135);
    TEST_ASSERT(def.stereo.epipolarTolerancePx == 150.0);
    TEST_ASSERT(!def.stereo.swapCameras);
    TEST_ASSERT(def.sourcePath.empty());

    // Missing file -> defaults, not an error
    bool loaded = true;
    AppConfig missing = AppConfig::loadFromFile(TestSandbox::path("does_not_exist.json"), &loaded);
    TEST_ASSERT(!loaded);
    TEST_ASSERT(missing.camera.exposureUs == 7812);

    // Partial file overrides only what it names
    const std::string path = TestSandbox::path("appconfig_partial.json");
    {
        std::ofstream out(path);
        out << R"({"camera": {"gain": 30}, "detector": {"intensityThreshold": 90}, "stereo": {"swapCameras": true}})";
    }
    AppConfig partial = AppConfig::loadFromFile(path, &loaded);
    TEST_ASSERT(loaded);
    TEST_ASSERT(partial.sourcePath == path);
    TEST_ASSERT(partial.camera.gain == 30);
    TEST_ASSERT(partial.camera.exposureUs == 7812);
    TEST_ASSERT(partial.detector.intensityThreshold == 90);
    TEST_ASSERT(partial.detector.clusterRadiusPx == 28.0);
    TEST_ASSERT(partial.stereo.swapCameras);
    TEST_ASSERT(!partial.describe().empty());

    // Malformed file -> defaults, logged, no throw
    const std::string bad = TestSandbox::path("appconfig_bad.json");
    {
        std::ofstream out(bad);
        out << "{ not json";
    }
    AppConfig broken = AppConfig::loadFromFile(bad, &loaded);
    TEST_ASSERT(!loaded);
    TEST_ASSERT(broken.camera.exposureUs == 7812);

    // Full round trip
    AppConfig again = AppConfig::fromJson(partial.toJson());
    TEST_ASSERT(again.camera.gain == 30 && again.detector.intensityThreshold == 90 && again.stereo.swapCameras);

    spdlog::info("[TEST] AppConfig load/merge/round-trip passed.");
}

GOLFSIM_TEST(DotClusterFinder) {
    DotClusterConfig cfg;
    cfg.intensityThreshold = 100;
    DotClusterFinder finder(cfg, 23.3);
    DotClusterResult res;

    // 1. Ideal ball: black field, 9 dots inside a 10 px cap -> one cluster at the truth
    cv::Mat ideal = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(ideal, cv::Point(667, 364));
    finder.find(ideal, res);
    TEST_ASSERT(res.clusters.size() == 1);
    TEST_ASSERT(res.clusters[0].dots.size() == 9);
    TEST_NEAR(res.clusters[0].centroid.x, 667.0, 0.5);
    TEST_NEAR(res.clusters[0].centroid.y, 364.0, 0.5);
    TEST_ASSERT(res.clusters[0].spreadPx > 6.0 && res.clusters[0].spreadPx < 47.0);
    TEST_ASSERT(res.rejectedDots.empty());
    TEST_ASSERT(res.rejectedClusters.empty());
    // Nominal ball box around the centroid
    TEST_ASSERT(res.clusters[0].boundingBox.width == 46);
    TEST_ASSERT(res.clusters[0].boundingBox.contains(cv::Point(667, 364)));

    // Diagnostics carry the accepted cluster in the FlightRecorder candidates shape
    nlohmann::json j = res.toJson();
    TEST_ASSERT(j["candidates"].size() == 1);
    TEST_ASSERT(j["candidates"][0]["accepted"] == true);
    TEST_ASSERT(j["candidates"][0]["dotCount"] == 9);
    TEST_ASSERT(j["candidates"][0]["markers"].size() == 9);
    TEST_ASSERT(j["threshold"] == 100);

    // 2. The specular reflection bar measured on the rig (19x59 px): rejected as a dot,
    //    never a cluster; the ball is still found exactly once
    cv::Mat withBar = ideal.clone();
    cv::rectangle(withBar, cv::Rect(684, 231, 19, 59), cv::Scalar(255), -1);
    finder.find(withBar, res);
    TEST_ASSERT(res.clusters.size() == 1);
    TEST_ASSERT(res.clusters[0].dots.size() == 9);
    TEST_ASSERT(res.rejectedDots.size() == 1);
    TEST_ASSERT(std::string(res.rejectedDots[0].reason).find("area too large") != std::string::npos ||
                std::string(res.rejectedDots[0].reason).find("elongated") != std::string::npos);

    // A thinner bar that passes the area gate is still caught by the aspect gate
    cv::Mat thinBar = ideal.clone();
    cv::rectangle(thinBar, cv::Rect(300, 100, 3, 30), cv::Scalar(255), -1);   // 90 px^2, aspect 10
    finder.find(thinBar, res);
    TEST_ASSERT(res.clusters.size() == 1);
    TEST_ASSERT(res.rejectedDots.size() == 1);
    TEST_ASSERT(std::string(res.rejectedDots[0].reason).find("elongated") != std::string::npos);

    // 3. Lone sparkle far from the ball: a cluster of one, rejected (too few dots)
    cv::Mat sparkle = ideal.clone();
    cv::circle(sparkle, cv::Point(200, 700), 2, cv::Scalar(240), -1);
    finder.find(sparkle, res);
    TEST_ASSERT(res.clusters.size() == 1);
    TEST_ASSERT(res.rejectedClusters.size() == 1);
    TEST_ASSERT(std::string(res.rejectedClusters[0].reason).find("too few") != std::string::npos);

    // 4. Two balls 200 px apart: two clusters, neither absorbs the other
    cv::Mat two = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(two, cv::Point(400, 400));
    drawDotBall(two, cv::Point(600, 400), 10, 7);
    finder.find(two, res);
    TEST_ASSERT(res.clusters.size() == 2);
    TEST_ASSERT(res.clusters[0].dots.size() == 9);   // most dots first
    TEST_ASSERT(res.clusters[1].dots.size() == 7);
    TEST_NEAR(res.clusters[0].centroid.x, 400.0, 0.5);
    TEST_NEAR(res.clusters[1].centroid.x, 600.0, 0.5);

    // 5. Merged dots: two glints 1 px apart form one blob -> still one dot, cluster still found
    cv::Mat merged = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(merged, cv::Point(500, 300), 10, 5);
    cv::circle(merged, cv::Point(500, 300), 2, cv::Scalar(235), -1);
    cv::circle(merged, cv::Point(503, 300), 2, cv::Scalar(235), -1);
    finder.find(merged, res);
    TEST_ASSERT(res.clusters.size() == 1);
    TEST_ASSERT(res.clusters[0].dots.size() == 5);

    // 6. Lit-up background: a 7x7 lattice of real-sized dots is one cluster of
    //    49 (> maxDotsPerCluster), rejected, not reported as a ball
    cv::Mat noisy = cv::Mat::zeros(800, 1280, CV_8UC1);
    for (int y = 0; y < 42; y += 6) {
        for (int x = 0; x < 42; x += 6) {
            cv::circle(noisy, cv::Point(500 + x, 300 + y), 2, cv::Scalar(255), -1);
        }
    }
    finder.find(noisy, res);
    TEST_ASSERT(res.clusters.empty());
    TEST_ASSERT(res.rejectedClusters.size() == 1);
    TEST_ASSERT(std::string(res.rejectedClusters[0].reason).find("too many") != std::string::npos);

    // Single-pixel sensor sparkle is below minDotArea: rejected as a dot
    cv::Mat pixel = cv::Mat::zeros(800, 1280, CV_8UC1);
    pixel.at<uint8_t>(100, 100) = 255;
    finder.find(pixel, res);
    TEST_ASSERT(res.clusters.empty());
    TEST_ASSERT(res.rejectedDots.size() == 1);
    TEST_ASSERT(std::string(res.rejectedDots[0].reason).find("too small") != std::string::npos);

    // 7. Empty and black frames
    finder.find(cv::Mat(), res);
    TEST_ASSERT(res.clusters.empty());
    finder.find(cv::Mat::zeros(800, 1280, CV_8UC1), res);
    TEST_ASSERT(res.clusters.empty() && res.rejectedDots.empty());

    // 8. Silhouette (old model): a filled disc is ONE huge blob, i.e. not a dots-only ball
    cv::Mat disc = cv::Mat::zeros(800, 1280, CV_8UC1);
    cv::circle(disc, cv::Point(640, 400), 23, cv::Scalar(200), -1);
    finder.find(disc, res);
    TEST_ASSERT(res.clusters.empty());
    TEST_ASSERT(res.rejectedDots.size() == 1);

    // 9. Config JSON round trip
    DotClusterConfig rt = DotClusterConfig::fromJson(cfg.toJson());
    TEST_ASSERT(rt.intensityThreshold == 100 && rt.clusterRadiusPx == 28.0 && rt.minDotsPerCluster == 3);

    spdlog::info("[TEST] DotClusterFinder passed (ideal, bar, sparkle, two balls, merged, noise, silhouette).");
}

GOLFSIM_TEST(DotClusterTracker) {
    DotClusterConfig cfg;
    cfg.intensityThreshold = 100;
    DotClusterTracker tracker(cfg, 23.3);

    cv::Mat frame = cv::Mat::zeros(800, 1280, CV_8UC1);
    drawDotBall(frame, cv::Point(831, 449));
    cv::rectangle(frame, cv::Rect(772, 325, 5, 56), cv::Scalar(255), -1);   // right-camera bar

    auto balls = tracker.detectBalls(frame);
    TEST_ASSERT(balls.size() == 1);
    TEST_NEAR(balls[0].centroid.x, 831.0, 0.5);
    TEST_NEAR(balls[0].centroid.y, 449.0, 0.5);
    TEST_ASSERT(balls[0].markers.size() == 9);          // the dots ARE the markers
    TEST_ASSERT(balls[0].markers[0].intensity >= 200.0);
    TEST_ASSERT(balls[0].boundingBox.contains(cv::Point(831, 449)));

    nlohmann::json diag = tracker.getLatestDiagnostics();
    TEST_ASSERT(diag["candidates"].size() == 1);
    TEST_ASSERT(diag["rejectedDotCount"] == 1);

    // IComputerVision polymorphism, and an empty frame yields nothing
    IComputerVision* base = &tracker;
    TEST_ASSERT(base->detectBalls(cv::Mat::zeros(800, 1280, CV_8UC1)).empty());

    spdlog::info("[TEST] DotClusterTracker passed.");
}
