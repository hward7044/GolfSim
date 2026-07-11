#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <iomanip>

#include "HAL/IUsbVideoDriver.hpp"
#include "HAL/V4L2Driver.hpp"
#include "HAL/MediaFoundationDriver.hpp"
#include "Camera/CameraRole.hpp"
#include "Camera/FrameSet.hpp"
#include "Camera/ICameraNode.hpp"
#include "Camera/OV9281CameraNode.hpp"
#include "Camera/PlaybackCameraNode.hpp"
#include "Camera/ICameraSystem.hpp"
#include "Camera/HardwareSyncedCameraSystem.hpp"
#include "Diagnostics/LogLevel.hpp"
#include "Diagnostics/GlobalLogger.hpp"
#include "Diagnostics/FlightRecorder.hpp"
#include "Math/LaunchData.hpp"
#include "Math/IBufferManager.hpp"
#include "Math/AtomicRingBuffer.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/OpticalGateTrigger.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/OpenCVMomentsTracker.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/EigenBallisticsEngine.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Math/TcpJsonTransmitter.hpp"
#include "Orchestration/SessionStateMachine.hpp"
#include "Orchestration/ThreadManager.hpp"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <filesystem>
#include <fstream>
#include <sstream>

const bool RUN_DEBUG_VIEWER = false;
void runCameraDebugViewer();

void runReplayViewer(const std::string& replayDir);

int main(int argc, char* argv[]) {
    // Parse replay argument if provided: --replay <dir> or -r <dir>
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--replay" || arg == "-r") && i + 1 < argc) {
            runReplayViewer(argv[i + 1]);
            return 0;
        }
    }
    // Determine compiler-specific C++ standard version
    long cpp_version = __cplusplus;
#ifdef _MSVC_LANG
    cpp_version = _MSVC_LANG;
#endif

    std::cout << "============================================" << std::endl;
    std::cout << "GolfSim Build Environment Verification" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "C++ Standard: " << cpp_version << " (e.g., 202002 for C++20)" << std::endl;
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
    std::cout << "Eigen Version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << "spdlog Version: " << SPDLOG_VER_MAJOR << "." << SPDLOG_VER_MINOR << "." << SPDLOG_VER_PATCH << std::endl;
    std::cout << "nlohmann/json Version: " << NLOHMANN_JSON_VERSION_MAJOR << "." << NLOHMANN_JSON_VERSION_MINOR << "." << NLOHMANN_JSON_VERSION_PATCH << std::endl;
    std::cout << "============================================" << std::endl;

    // Test nlohmann/json
    nlohmann::json test_json;
    test_json["status"] = "OK";
    test_json["message"] = "Build environment is fully operational!";
    std::cout << "JSON Test Output: " << test_json.dump() << std::endl;

    // Test spdlog
    spdlog::info("spdlog is working correctly.");

    // Test Eigen Matrix multiplication
    Eigen::Matrix2d mat;
    mat << 1, 2, 3, 4;
    std::cout << "Eigen Matrix multiplication test:\n" << mat * mat << std::endl;

    // Test OpenCV Matrix creation
    cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
    std::cout << "OpenCV Matrix created successfully. Dimensions: " << image.rows << "x" << image.cols << std::endl;

    std::cout << "============================================" << std::endl;
    
    // Run C++ Math Verification Tests
    void runMathTests();
    runMathTests(); // Run verification tests on startup

    std::cout << "Verification completed successfully!" << std::endl;

    if (RUN_DEBUG_VIEWER) {
        runCameraDebugViewer();
        return 0;
    }

    // -------------------------------------------------------------------------
    // Production Launch Monitor Pipeline
    // -------------------------------------------------------------------------
    std::cout << "\n============================================" << std::endl;
    std::cout << "Starting Production Launch Monitor Pipeline" << std::endl;
    std::cout << "============================================" << std::endl;

    // Configure spdlog to write to both console and build/session.log
    try {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("build/session.log", true);
        spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));
        spdlog::set_level(spdlog::level::info);
        spdlog::flush_on(spdlog::level::info);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << "Log initialization failed: " << ex.what() << std::endl;
    }

    spdlog::info("[System] Initializing camera drivers...");
    MediaFoundationDriver::logConnectedDevices();

    // 2. Initialize the MediaFoundationDrivers
    auto usbLeft = std::make_unique<MediaFoundationDriver>(0);
    bool leftOk = usbLeft->initialize();
    
    auto usbRight = std::make_unique<MediaFoundationDriver>(1);
    bool rightOk = usbRight->initialize();

    if (!leftOk && !rightOk) {
        spdlog::warn("[System] Failed to initialize camera hardware (expected in emulation/test environments). Clean exit.");
        return 0;
    }

    auto cameraSystem = std::make_shared<HardwareSyncedCameraSystem>();
    uint32_t width = 1280;
    uint32_t height = 800;

    if (leftOk) {
        width = usbLeft->getFrameWidth();
        height = usbLeft->getFrameHeight();
        auto cameraNodeLeft = std::make_shared<OV9281CameraNode>(std::move(usbLeft), CameraRole::STEREO_LEFT);
        cameraSystem->addCameraNode(cameraNodeLeft);
        spdlog::info("[System] Successfully registered Left camera ({}x{})", width, height);
    }

    if (rightOk) {
        width = usbRight->getFrameWidth();
        height = usbRight->getFrameHeight();
        auto cameraNodeRight = std::make_shared<OV9281CameraNode>(std::move(usbRight), CameraRole::STEREO_RIGHT);
        cameraSystem->addCameraNode(cameraNodeRight);
        spdlog::info("[System] Successfully registered Right camera ({}x{})", width, height);
    }

    // Queue buffer manager (capacity of 16 FrameSets)
    auto buffer = std::make_shared<AtomicRingBuffer<FrameSet, 16>>();

    // Pipeline components initialization:
    // - Left camera optical gate trigger (Region of Interest, min Ball pixels, diff threshold, EMA alpha)
    // - Image Moments tracker (Ball threshold, Marker threshold, min Area, max Area, min Circularity)
    // - Stereo Triangulator (Uses default horizontal calibration)
    // - Kinematics physics engine
    // - Local network TCP transmitter (Target loopback, port 9002)
    cv::Rect triggerRoi(100, 100, 400, 400);
    auto trigger = OpticalGateTrigger(triggerRoi, 150, 25, 0.05);
    auto vision = OpenCVMomentsTracker(80, 240, 80.0, 2500.0, 0.5);
    auto spatial = StereoTriangulator();
    auto kinematics = EigenBallisticsEngine();
    auto network = TcpJsonTransmitter("127.0.0.1", 9002);

    auto stateMachine = std::make_shared<ConcreteSSM>(
        trigger,
        vision,
        spatial,
        kinematics,
        network
    );

    auto threadManager = std::make_shared<ThreadManager>(
        cameraSystem,
        buffer,
        stateMachine
    );

    spdlog::info("[System] Starting background acquisition and tracking threads...");
    threadManager->startProducerThread();
    threadManager->startConsumerThread();

    spdlog::info("[System] System is online and monitoring. Press Enter to shutdown.");
    std::cin.get();

    spdlog::info("[System] Shutting down threads...");
    threadManager->stop();
    spdlog::info("[System] Shutdown completed cleanly.");
    return 0;
}

// -------------------------------------------------------------------------
// Live Camera Setup and Test Viewer (RUN_DEBUG_VIEWER = true)
// -------------------------------------------------------------------------
void runCameraDebugViewer() {
    std::cout << "\n============================================" << std::endl;
    std::cout << "Starting Live Camera Hookup and Test" << std::endl;
    std::cout << "============================================" << std::endl;

    // 1. Enumerate and log all connected video capture devices on the system
    MediaFoundationDriver::logConnectedDevices();

    // 2. Initialize the MediaFoundationDrivers
    std::cout << "\nInitializing camera driver 0 (Left)..." << std::endl;
    auto usbDriverLeft = std::make_unique<MediaFoundationDriver>(0);
    bool leftOk = usbDriverLeft->initialize();
    
    std::cout << "Initializing camera driver 1 (Right)..." << std::endl;
    auto usbDriverRight = std::make_unique<MediaFoundationDriver>(1);
    bool rightOk = usbDriverRight->initialize();

    if (!leftOk && !rightOk) {
        std::cerr << "Failed to initialize either camera. Are they connected?" << std::endl;
        return;
    }

    // 3 & 4. Create and Register the nodes with the camera system
    HardwareSyncedCameraSystem cameraSystem;
    uint32_t width = 1280;
    uint32_t height = 800;

    if (leftOk) {
        width = usbDriverLeft->getFrameWidth();
        height = usbDriverLeft->getFrameHeight();
        auto cameraNodeLeft = std::make_shared<OV9281CameraNode>(std::move(usbDriverLeft), CameraRole::STEREO_LEFT);
        cameraSystem.addCameraNode(cameraNodeLeft);
        std::cout << "Successfully initialized Left camera! Resolution: " << width << "x" << height << std::endl;
    } else {
        std::cout << "Left camera was not detected/initialized." << std::endl;
    }

    if (rightOk) {
        width = usbDriverRight->getFrameWidth();
        height = usbDriverRight->getFrameHeight();
        auto cameraNodeRight = std::make_shared<OV9281CameraNode>(std::move(usbDriverRight), CameraRole::STEREO_RIGHT);
        cameraSystem.addCameraNode(cameraNodeRight);
        std::cout << "Successfully initialized Right camera! Resolution: " << width << "x" << height << std::endl;
    } else {
        std::cout << "Right camera was not detected/initialized." << std::endl;
    }

    // 5. Pre-allocate the FrameSet buffer
    FrameSet frameSet;
    frameSet.preallocate(width, height);

    enum ViewMode { VIEW_BOTH, VIEW_LEFT_ONLY, VIEW_RIGHT_ONLY };
    int currentMode = VIEW_BOTH;

    std::cout << "\nStarting live video display. Controls:" << std::endl;
    std::cout << "  - Press TAB or 'v' inside the video window to cycle views (Both -> Left -> Right)" << std::endl;
    std::cout << "  - Press ESC to quit" << std::endl;

    std::string windowName = "Stereo Cameras Live Feed";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    auto start_time = std::chrono::steady_clock::now();
    uint32_t frame_count = 0;
    double fps = 0.0;

    while (true) {
        if (!cameraSystem.captureSynchronizedFrames(frameSet)) {
            std::cerr << "Failed to capture frames!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

        frame_count++;

        // Measure FPS
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() >= 1.0) {
            fps = frame_count / elapsed.count();
            frame_count = 0;
            start_time = current_time;
        }

        // Get the frames
        cv::Mat leftFrame = frameSet.getFrame(CameraRole::STEREO_LEFT);
        cv::Mat rightFrame = frameSet.getFrame(CameraRole::STEREO_RIGHT);

        cv::Scalar meanLeft = leftFrame.empty() ? cv::Scalar(0) : cv::mean(leftFrame);
        cv::Scalar meanRight = rightFrame.empty() ? cv::Scalar(0) : cv::mean(rightFrame);

        // Print stats periodically to console
        if (frame_count % 30 == 0) {
            std::cout << "Frame Stats: L=" 
                      << (leftFrame.empty() ? "N/A" : std::to_string(leftFrame.cols) + "x" + std::to_string(leftFrame.rows))
                      << " (Avg=" << std::fixed << std::setprecision(1) << meanLeft[0] << ")"
                      << " | R=" 
                      << (rightFrame.empty() ? "N/A" : std::to_string(rightFrame.cols) + "x" + std::to_string(rightFrame.rows))
                      << " (Avg=" << meanRight[0] << ")"
                      << " | ViewMode=" << (currentMode == VIEW_BOTH ? "BOTH" : (currentMode == VIEW_LEFT_ONLY ? "LEFT" : "RIGHT"))
                      << " | FPS=" << std::setprecision(2) << fps << "\r" << std::flush;
        }

        // Draw overlays
        std::string fpsText = "FPS: " + std::to_string(fps).substr(0, 5);
        cv::Mat displayLeft, displayRight;

        if (!leftFrame.empty()) {
            cv::cvtColor(leftFrame, displayLeft, cv::COLOR_GRAY2BGR);
            std::string intTextL = "Intensity: " + std::to_string(meanLeft[0]).substr(0, 4);
            cv::putText(displayLeft, "LEFT " + fpsText, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(displayLeft, intTextL, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }

        if (!rightFrame.empty()) {
            cv::cvtColor(rightFrame, displayRight, cv::COLOR_GRAY2BGR);
            std::string intTextR = "Intensity: " + std::to_string(meanRight[0]).substr(0, 4);
            cv::putText(displayRight, "RIGHT " + fpsText, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(displayRight, intTextR, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }

        // Build composite image based on active view mode
        cv::Mat frameToDraw;
        if (currentMode == VIEW_BOTH) {
            if (!displayLeft.empty() && !displayRight.empty()) {
                cv::Mat resizedLeft, resizedRight;
                cv::resize(displayLeft, resizedLeft, cv::Size(640, 400));
                cv::resize(displayRight, resizedRight, cv::Size(640, 400));
                cv::hconcat(resizedLeft, resizedRight, frameToDraw);
            } else if (!displayLeft.empty()) {
                frameToDraw = displayLeft;
            } else if (!displayRight.empty()) {
                frameToDraw = displayRight;
            }
        } else if (currentMode == VIEW_LEFT_ONLY) {
            if (!displayLeft.empty()) {
                frameToDraw = displayLeft;
            } else {
                frameToDraw = cv::Mat::zeros(400, 640, CV_8UC3);
                cv::putText(frameToDraw, "LEFT CAMERA OFFLINE", cv::Point(120, 200), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            }
        } else if (currentMode == VIEW_RIGHT_ONLY) {
            if (!displayRight.empty()) {
                frameToDraw = displayRight;
            } else {
                frameToDraw = cv::Mat::zeros(400, 640, CV_8UC3);
                cv::putText(frameToDraw, "RIGHT CAMERA OFFLINE", cv::Point(120, 200), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            }
        }

        if (!frameToDraw.empty()) {
            cv::imshow(windowName, frameToDraw);
        }

        // Break on ESC key, toggle views on TAB (9) or v/V (118/86)
        int key = cv::waitKey(1);
        if (key == 27) {
            break;
        } else if (key == 9 || key == 118 || key == 86) {
            currentMode = (currentMode + 1) % 3;
            std::cout << "\nSwitched View Mode to: " 
                      << (currentMode == VIEW_BOTH ? "BOTH (50/50)" : (currentMode == VIEW_LEFT_ONLY ? "LEFT ONLY" : "RIGHT ONLY")) 
                      << std::endl;
        }
    }

    std::cout << "\nStopping acquisition..." << std::endl;
    cv::destroyAllWindows();
    cameraSystem.shutdown();
    std::cout << "Shutdown completed cleanly." << std::endl;
}

void runReplayViewer(const std::string& replayDir) {
    namespace fs = std::filesystem;
    fs::path dir(replayDir);
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        std::cerr << "Error: Replay directory does not exist: " << replayDir << std::endl;
        return;
    }

    fs::path metaPath = dir / "metadata.json";
    if (!fs::exists(metaPath)) {
        std::cerr << "Error: metadata.json not found in " << replayDir << std::endl;
        return;
    }

    nlohmann::json meta;
    try {
        std::ifstream in(metaPath.string());
        if (!in.is_open()) throw std::runtime_error("Could not open file");
        in >> meta;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing metadata.json: " << e.what() << std::endl;
        return;
    }

    std::string shotId = meta.value("shotId", "unknown");
    std::cout << "============================================" << std::endl;
    std::cout << "Loading Shot Replay: " << shotId << std::endl;
    if (meta.contains("kinematics")) {
        auto k = meta["kinematics"];
        std::cout << "Kinematics Solved:" << std::endl;
        std::cout << "  - Speed: " << k.value("ballSpeed_mph", 0.0) << " mph" << std::endl;
        std::cout << "  - VLA: " << k.value("verticalLaunchAngle_deg", 0.0) << " deg" << std::endl;
        std::cout << "  - HLA: " << k.value("horizontalLaunchAngle_deg", 0.0) << " deg" << std::endl;
        std::cout << "  - Spin: " << k.value("spinRPM", 0.0) << " RPM" << std::endl;
    }
    std::cout << "============================================" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  - SPACE : Pause / Play auto-playback" << std::endl;
    std::cout << "  - d / Arrow Right : Step forward 1 frame" << std::endl;
    std::cout << "  - a / Arrow Left  : Step backward 1 frame" << std::endl;
    std::cout << "  - 'o'   : Toggle diagnostic overlays ON/OFF" << std::endl;
    std::cout << "  - ESC   : Exit Replay Viewer" << std::endl;

    auto framesJson = meta["frames"];
    size_t frameCount = framesJson.size();
    if (frameCount == 0) {
        std::cerr << "Replay contains zero frames!" << std::endl;
        return;
    }

    std::string windowName = "GolfSim Interactive Shot Replay - " + shotId;
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

    size_t currentIndex = 0;
    bool playing = false;
    bool showOverlays = true;

    while (true) {
        std::ostringstream nameOss;
        nameOss << std::setw(3) << std::setfill('0') << currentIndex << ".png";
        std::string filename = nameOss.str();

        cv::Mat leftImg, rightImg;
        if (showOverlays) {
            leftImg = cv::imread((dir / "annotated" / ("left_" + filename)).string());
            rightImg = cv::imread((dir / "annotated" / ("right_" + filename)).string());
        } else {
            leftImg = cv::imread((dir / "raw" / ("left_" + filename)).string());
            rightImg = cv::imread((dir / "raw" / ("right_" + filename)).string());
            if (!leftImg.empty() && leftImg.channels() == 1) {
                cv::cvtColor(leftImg, leftImg, cv::COLOR_GRAY2BGR);
            }
            if (!rightImg.empty() && rightImg.channels() == 1) {
                cv::cvtColor(rightImg, rightImg, cv::COLOR_GRAY2BGR);
            }
        }

        if (leftImg.empty() && rightImg.empty()) {
            std::cerr << "\nFailed to load frame " << currentIndex << std::endl;
            break;
        }

        cv::Mat displayLeft, displayRight;
        if (!leftImg.empty()) {
            cv::resize(leftImg, displayLeft, cv::Size(640, 400));
        } else {
            displayLeft = cv::Mat::zeros(400, 640, CV_8UC3);
        }

        if (!rightImg.empty()) {
            cv::resize(rightImg, displayRight, cv::Size(640, 400));
        } else {
            displayRight = cv::Mat::zeros(400, 640, CV_8UC3);
        }

        cv::Mat composite;
        cv::hconcat(displayLeft, displayRight, composite);

        std::string statusText = "Frame " + std::to_string(currentIndex + 1) + " / " + std::to_string(frameCount) +
                                 " | " + (playing ? "PLAYING" : "PAUSED") +
                                 " | Overlays: " + (showOverlays ? "ON" : "OFF");
        cv::rectangle(composite, cv::Rect(5, 5, 450, 30), cv::Scalar(0, 0, 0), -1);
        cv::putText(composite, statusText, cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

        cv::imshow(windowName, composite);

        int waitTime = playing ? 100 : 0;
        int key = cv::waitKey(waitTime);

        if (key == 27) { // ESC
            break;
        } else if (key == ' ') {
            playing = !playing;
        } else if (key == 'o' || key == 'O') {
            showOverlays = !showOverlays;
        } else if (key == 'd' || key == 'D' || key == 2424832 || key == 65363 || key == 79) { // right
            currentIndex = (currentIndex + 1) % frameCount;
        } else if (key == 'a' || key == 'A' || key == 2424830 || key == 65361 || key == 80) { // left
            currentIndex = (currentIndex + frameCount - 1) % frameCount;
        } else if (playing) {
            currentIndex = (currentIndex + 1) % frameCount;
        }
    }

    cv::destroyWindow(windowName);
}
