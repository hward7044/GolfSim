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

int main() {
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
    // runMathTests(); // Commented out to focus on camera hookup and streaming test

    std::cout << "Verification completed successfully!" << std::endl;

    std::cout << "\n============================================" << std::endl;
    std::cout << "Starting Live Camera Hookup and Test" << std::endl;
    std::cout << "============================================" << std::endl;

    // 1. Enumerate and log all connected video capture devices on the system
    MediaFoundationDriver::logConnectedDevices();

    // 2. Initialize the MediaFoundationDriver for device index 0
    std::cout << "\nInitializing camera driver for device index 0..." << std::endl;
    auto usbDriver = std::make_unique<MediaFoundationDriver>(0);
    
    if (!usbDriver->initialize()) {
        std::cerr << "Failed to initialize MediaFoundationDriver for device 0. Is the camera connected?" << std::endl;
        return -1;
    }

    uint32_t width = usbDriver->getFrameWidth();
    uint32_t height = usbDriver->getFrameHeight();
    std::cout << "Successfully initialized driver! Resolution: " 
              << width << "x" << height << std::endl;

    // 3. Create the OV9281CameraNode wrapping the driver
    auto cameraNode = std::make_shared<OV9281CameraNode>(std::move(usbDriver), CameraRole::STEREO_LEFT);

    // 4. Register the node with the camera system
    HardwareSyncedCameraSystem cameraSystem;
    cameraSystem.addCameraNode(cameraNode);

    // 5. Pre-allocate the FrameSet buffer
    FrameSet frameSet;
    frameSet.preallocate(width, height);

    std::cout << "\nStarting live video display. Press ESC in the video window to quit." << std::endl;

    cv::namedWindow("OV9281 Camera Live Feed", cv::WINDOW_AUTOSIZE);

    auto start_time = std::chrono::steady_clock::now();
    uint32_t frame_count = 0;
    double fps = 0.0;

    while (true) {
        if (!cameraSystem.captureSynchronizedFrames(frameSet)) {
            std::cerr << "Failed to capture frame!" << std::endl;
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

        // Get the frame
        cv::Mat frame = frameSet.getFrame(CameraRole::STEREO_LEFT);
        if (!frame.empty()) {
            // Calculate diagnostics: mean pixel intensity
            cv::Scalar meanVal = cv::mean(frame);
            
            // Print diagnostics to console (throttled to avoid spamming too fast)
            if (frame_count % 30 == 0) {
                std::cout << "Frame Stats: Resolution=" << frame.cols << "x" << frame.rows 
                          << " | Avg Pixel Intensity=" << std::fixed << std::setprecision(1) << meanVal[0]
                          << " | Live FPS=" << std::setprecision(2) << fps << "\r" << std::flush;
            }

            // Draw overlay text on the image
            cv::Mat displayFrame;
            // Convert to color so we can draw colorful text
            cv::cvtColor(frame, displayFrame, cv::COLOR_GRAY2BGR);

            std::string fpsText = "FPS: " + std::to_string(fps).substr(0, 5);
            std::string intText = "Intensity: " + std::to_string(meanVal[0]).substr(0, 4);
            cv::putText(displayFrame, fpsText, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(displayFrame, intText, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Display frame
            cv::imshow("OV9281 Camera Live Feed", displayFrame);
        }

        // Break on ESC key
        int key = cv::waitKey(1);
        if (key == 27) {
            break;
        }
    }

    std::cout << "\nStopping acquisition..." << std::endl;
    cv::destroyAllWindows();
    cameraSystem.shutdown();

    std::cout << "Shutdown completed cleanly." << std::endl;
    return 0;
}
