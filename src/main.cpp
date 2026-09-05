#include <Eigen/Core>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#if __has_include(<opencv2/geometry.hpp>)
#include <opencv2/geometry.hpp>
#endif
#include <spdlog/spdlog.h>
#include <thread>

#include "Camera/CameraRole.hpp"
#include "Camera/FrameSet.hpp"
#include "Camera/HardwareSyncedCameraSystem.hpp"
#include "Camera/ICameraNode.hpp"
#include "Camera/ICameraSystem.hpp"
#include "Camera/OV9281CameraNode.hpp"
#include "Camera/PlaybackCameraNode.hpp"
#include "Diagnostics/FlightRecorder.hpp"
#include "Diagnostics/GlobalLogger.hpp"
#include "Diagnostics/LogLevel.hpp"
#include "HAL/IUsbVideoDriver.hpp"
#ifdef _WIN32
#include "HAL/MediaFoundationDriver.hpp"
#include "HAL/Win32Serial.hpp"
#else
#include "HAL/V4L2Driver.hpp"
#endif
#include "Math/AtomicRingBuffer.hpp"
#include "Math/BallPresenceTrigger.hpp"
#include "Math/EigenBallisticsEngine.hpp"
#include "Math/IBufferManager.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/IKinematicsSolver.hpp"
#include "Math/INetworkTransmitter.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/LaunchData.hpp"
#include "Math/OpenCVMomentsTracker.hpp"
#include "Math/StereoBallTrackerTrigger.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/TcpJsonTransmitter.hpp"
#include "Orchestration/SessionStateMachine.hpp"
#include "Orchestration/ThreadManager.hpp"
#include <filesystem>
#include <fstream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sstream>

const bool RUN_DEBUG_VIEWER = false;
void runCameraDebugViewer(int leftCamIdx, int rightCamIdx, const std::string& comPort = "COM3");

void runReplayViewer(const std::string &replayDir);

int main(int argc, char *argv[]) {
  bool streamMode = false;
  int streamFrames = 50;
  int leftCamIdx = 1;
  int rightCamIdx = 0;
  std::string comPort = "COM3";

  bool liveMode = false;

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "--replay" || arg == "-r") && i + 1 < argc) {
      runReplayViewer(argv[i + 1]);
      return 0;
    }
    if (arg == "--live" || arg == "-l" || arg == "--strobe" || arg == "--ir-debug") {
      liveMode = true;
    }
    if (arg == "--stream" || arg == "--record-stream" || arg == "-s") {
      streamMode = true;
    }
    if (arg == "--frames" && i + 1 < argc) {
      streamFrames = std::atoi(argv[i + 1]);
      if (streamFrames <= 0) streamFrames = 50;
    }
    if (arg == "--left-cam" && i + 1 < argc) {
      leftCamIdx = std::atoi(argv[++i]);
    }
    if (arg == "--right-cam" && i + 1 < argc) {
      rightCamIdx = std::atoi(argv[++i]);
    }
    if (arg == "--swap-cameras" || arg == "--swap") {
      std::swap(leftCamIdx, rightCamIdx);
    }
    if (arg == "--com" && i + 1 < argc) {
      comPort = argv[++i];
    }
  }

  if (liveMode) {
    runCameraDebugViewer(leftCamIdx, rightCamIdx, comPort);
    return 0;
  }
  // Determine compiler-specific C++ standard version
  long cpp_version = __cplusplus;
#ifdef _MSVC_LANG
  cpp_version = _MSVC_LANG;
#endif

  std::cout << "============================================" << std::endl;
  std::cout << "GolfSim Build Environment Verification" << std::endl;
  std::cout << "============================================" << std::endl;
  std::cout << "C++ Standard: " << cpp_version << " (e.g., 202002 for C++20)"
            << std::endl;
  std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
  std::cout << "Eigen Version: " << EIGEN_WORLD_VERSION << "."
            << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
  std::cout << "spdlog Version: " << SPDLOG_VER_MAJOR << "." << SPDLOG_VER_MINOR
            << "." << SPDLOG_VER_PATCH << std::endl;
  std::cout << "nlohmann/json Version: " << NLOHMANN_JSON_VERSION_MAJOR << "."
            << NLOHMANN_JSON_VERSION_MINOR << "." << NLOHMANN_JSON_VERSION_PATCH
            << std::endl;
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
  std::cout << "OpenCV Matrix created successfully. Dimensions: " << image.rows
            << "x" << image.cols << std::endl;

  std::cout << "============================================" << std::endl;

  // Run C++ Math Verification Tests
  void runMathTests();
  runMathTests(); // Run verification tests on startup

  std::cout << "Verification completed successfully!" << std::endl;

  if (RUN_DEBUG_VIEWER) {
    runCameraDebugViewer(leftCamIdx, rightCamIdx, comPort);
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
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        "build/session.log", true);
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_on(spdlog::level::info);
  } catch (const spdlog::spdlog_ex &ex) {
    std::cerr << "Log initialization failed: " << ex.what() << std::endl;
  }

  auto cameraSystem = std::make_shared<HardwareSyncedCameraSystem>();

#ifdef _WIN32
  spdlog::info("[System] Initializing camera drivers...");
  MediaFoundationDriver::logConnectedDevices();

  // 2. Initialize the MediaFoundationDrivers with correct hardware device mapping
  spdlog::info("[System] Mapping Left Camera to Index {}, Right Camera to Index {}", leftCamIdx, rightCamIdx);
  auto usbLeft = std::make_unique<MediaFoundationDriver>(leftCamIdx);
  bool leftOk = usbLeft->initialize();

  auto usbRight = std::make_unique<MediaFoundationDriver>(rightCamIdx);
  bool rightOk = usbRight->initialize();

  if (!leftOk && !rightOk) {
    spdlog::warn("[System] Failed to initialize camera hardware (expected in "
                 "emulation/test environments). Clean exit.");
    return 0;
  }

  uint32_t width = 1280;
  uint32_t height = 800;

  if (leftOk) {
    width = usbLeft->getFrameWidth();
    height = usbLeft->getFrameHeight();
    auto cameraNodeLeft = std::make_shared<OV9281CameraNode>(
        std::move(usbLeft), CameraRole::STEREO_LEFT);
    cameraSystem->addCameraNode(cameraNodeLeft);
    spdlog::info("[System] Successfully registered Left camera ({}x{})", width,
                 height);
  }

  if (rightOk) {
    width = usbRight->getFrameWidth();
    height = usbRight->getFrameHeight();
    auto cameraNodeRight = std::make_shared<OV9281CameraNode>(
        std::move(usbRight), CameraRole::STEREO_RIGHT);
    cameraSystem->addCameraNode(cameraNodeRight);
    spdlog::info("[System] Successfully registered Right camera ({}x{})", width,
                 height);
  }
#else
  spdlog::info("[System] Initializing camera drivers...");
  spdlog::warn("[System] Hardware camera drivers (MediaFoundation) are only supported on Windows.");
  spdlog::warn("[System] Failed to initialize camera hardware (expected in emulation/test environments). Clean exit.");
  return 0;
#endif

  // Queue buffer manager (capacity of 16 FrameSets)
  auto buffer = std::make_shared<AtomicRingBuffer<FrameSet, 16>>();

  // Pipeline components initialization:
  // - Left camera optical gate trigger (Region of Interest, min Ball pixels,
  // diff threshold, EMA alpha)
  // - Image Moments tracker (Ball threshold, Marker threshold, min Area, max
  // Area, min Circularity)
  // - Stereo Triangulator (Uses default horizontal calibration)
  // - Kinematics physics engine
  // - Local network TCP transmitter (Target loopback, port 9002)
  // Stereoscopic Ball-Locked Tracker: Fine-tuned floor tee zone ROI (x=350..950, y=440..750)
  cv::Rect searchRoiLeft(350, 440, 600, 310);
  cv::Rect searchRoiRight(350, 440, 600, 310);
  auto trigger = StereoBallTrackerTrigger(
      StereoCalibration(), searchRoiLeft, searchRoiRight, 15.0, 85.0, 0.25,
      120, 65.0, 256, 0.9144, 5, 4, 4.0, 0.04, 150.0, 8500.0);
  auto vision = OpenCVMomentsTracker(120, 240, 80.0, 2500.0, 0.5);
  auto spatial = StereoTriangulator();
  auto kinematics = EigenBallisticsEngine();
  auto network = TcpJsonTransmitter("127.0.0.1", 9002);

  auto stateMachine = std::make_shared<ConcreteSSM>(trigger, vision, spatial,
                                                    kinematics, network);

  if (streamMode) {
    spdlog::info("[System] Stream Recording Mode Enabled! Chunk size: {} frames", streamFrames);
    stateMachine->setStreamRecordingMode(true, streamFrames);
  }

  auto threadManager =
      std::make_shared<ThreadManager>(cameraSystem, buffer, stateMachine);

  spdlog::info(
      "[System] Starting background acquisition and tracking threads...");
  threadManager->startProducerThread();
  threadManager->startConsumerThread();

  spdlog::info(
      "[System] System is online and monitoring. Press Enter to shutdown.");
  std::cin.get();

  spdlog::info("[System] Shutting down threads...");
  threadManager->stop();
  spdlog::info("[System] Shutdown completed cleanly.");
  return 0;
}

// -------------------------------------------------------------------------
// Live Camera Setup and Test Viewer (RUN_DEBUG_VIEWER = true)
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// Live Camera Setup and IR Strobe Debug Viewer
// -------------------------------------------------------------------------
void runCameraDebugViewer(int leftCamIdx, int rightCamIdx, const std::string& comPort) {
#ifdef _WIN32
  std::cout << "\n============================================" << std::endl;
  std::cout << "Starting Live Camera & IR Strobe Debug Viewer" << std::endl;
  std::cout << "============================================" << std::endl;

  Win32Serial serial;
  if (serial.open(comPort, 115200)) {
      std::cout << "Successfully connected to Arduino on " << comPort << std::endl;
  } else {
      std::cerr << "Warning: Could not open Serial port " << comPort << ". Hardware strobe testing disabled." << std::endl;
  }

  // 1. Enumerate connected video capture devices
  MediaFoundationDriver::logConnectedDevices();

  // 2. Initialize MediaFoundationDrivers with user-specified indices
  std::cout << "\nInitializing Left camera (Index " << leftCamIdx << ")..." << std::endl;
  auto usbDriverLeft = std::make_unique<MediaFoundationDriver>(leftCamIdx);
  bool leftOk = usbDriverLeft->initialize();

  std::cout << "Initializing Right camera (Index " << rightCamIdx << ")..." << std::endl;
  auto usbDriverRight = std::make_unique<MediaFoundationDriver>(rightCamIdx);
  bool rightOk = usbDriverRight->initialize();

  if (!leftOk && !rightOk) {
    std::cerr << "Failed to initialize cameras. Check USB connection." << std::endl;
    return;
  }

  HardwareSyncedCameraSystem cameraSystem;
  uint32_t width = 1280;
  uint32_t height = 800;

  std::shared_ptr<OV9281CameraNode> nodeL = nullptr;
  std::shared_ptr<OV9281CameraNode> nodeR = nullptr;

  if (leftOk) {
    width = usbDriverLeft->getFrameWidth();
    height = usbDriverLeft->getFrameHeight();
    nodeL = std::make_shared<OV9281CameraNode>(
        std::move(usbDriverLeft), CameraRole::STEREO_LEFT);
    cameraSystem.addCameraNode(nodeL);
    nodeL->enableHardwareStrobeMode();
    std::cout << "Successfully initialized Left camera (" << width << "x" << height << ") [Strobe Enabled]" << std::endl;
  }

  if (rightOk) {
    width = usbDriverRight->getFrameWidth();
    height = usbDriverRight->getFrameHeight();
    nodeR = std::make_shared<OV9281CameraNode>(
        std::move(usbDriverRight), CameraRole::STEREO_RIGHT);
    cameraSystem.addCameraNode(nodeR);
    nodeR->enableHardwareStrobeMode();
    std::cout << "Successfully initialized Right camera (" << width << "x" << height << ") [Strobe Enabled]" << std::endl;
  }

  FrameSet frameSet;
  frameSet.preallocate(width, height);

  enum ViewMode { VIEW_BOTH, VIEW_LEFT_ONLY, VIEW_RIGHT_ONLY, VIEW_GLINT_HIGHLIGHT, VIEW_THRESHOLD_MASK };
  int currentMode = VIEW_BOTH;
  int activeThreshold = 200;
  bool strobeOn = true;
  int expPresetIdx = 2; // Default 2ms
  const int expPresetsUs[] = { 500, 1000, 2000, 5000, 10000 };
  const int numExpPresets = 5;

  std::cout << "\n=======================================================" << std::endl;
  std::cout << "GOLFSIM IR STROBE DEBUGGER CONTROLS:" << std::endl;
  std::cout << "  - Press '1' : Turn IR Illumination CONTINUOUSLY ON ('1')" << std::endl;
  std::cout << "  - Press '0' : Turn IR Illumination OFF ('0')" << std::endl;
  std::cout << "  - Press 's' or 'f' : Fire 300ms IR Strobe Burst ('F')" << std::endl;
  std::cout << "  - Press 'e' : Cycle Exposure (500us -> 1ms -> 2ms -> 5ms -> 10ms)" << std::endl;
  std::cout << "  - Press 'v' : Cycle View Modes (Both -> Left -> Right -> Glints -> Threshold Mask)" << std::endl;
  std::cout << "  - Press '+' / '-' : Adjust Glint Threshold (Current: " << activeThreshold << ")" << std::endl;
  std::cout << "  - Press ESC / 'q' : Exit Debugger" << std::endl;
  std::cout << "=======================================================\n" << std::endl;

  std::string windowName = "GolfSim IR Strobe Debugger";
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);

  auto start_time = std::chrono::steady_clock::now();
  uint32_t frame_count = 0;
  double fps = 0.0;

  while (true) {
    if (!cameraSystem.captureSynchronizedFrames(frameSet)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    frame_count++;

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    if (elapsed.count() >= 1.0) {
      fps = frame_count / elapsed.count();
      frame_count = 0;
      start_time = current_time;
    }

    cv::Mat leftFrame = frameSet.getFrame(CameraRole::STEREO_LEFT);
    cv::Mat rightFrame = frameSet.getFrame(CameraRole::STEREO_RIGHT);

    cv::Scalar meanLeft = leftFrame.empty() ? cv::Scalar(0) : cv::mean(leftFrame);
    cv::Scalar meanRight = rightFrame.empty() ? cv::Scalar(0) : cv::mean(rightFrame);

    cv::Mat displayLeft, displayRight;

    std::string strobeStatusStr = strobeOn ? "STROBE: ON [Pin active]" : "STROBE: OFF";
    std::string expStr = "Exp: " + std::to_string(expPresetsUs[expPresetIdx]) + "us";

    if (!leftFrame.empty()) {
      cv::cvtColor(leftFrame, displayLeft, cv::COLOR_GRAY2BGR);
      if (currentMode == VIEW_GLINT_HIGHLIGHT || currentMode == VIEW_THRESHOLD_MASK) {
        cv::Mat mask;
        cv::threshold(leftFrame, mask, activeThreshold, 255, cv::THRESH_BINARY);
        if (currentMode == VIEW_THRESHOLD_MASK) {
          cv::cvtColor(mask, displayLeft, cv::COLOR_GRAY2BGR);
        } else {
          // Highlight glints in bright red
          std::vector<std::vector<cv::Point>> contours;
          cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
          for (const auto& c : contours) {
            cv::Rect r = cv::boundingRect(c);
            cv::rectangle(displayLeft, r, cv::Scalar(0, 0, 255), 2);
            cv::circle(displayLeft, cv::Point(r.x + r.width/2, r.y + r.height/2), r.width/2 + 4, cv::Scalar(0, 255, 255), 2);
          }
        }
      }
      cv::putText(displayLeft, "LEFT | " + strobeStatusStr, cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, strobeOn ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
      cv::putText(displayLeft, expStr + " | Thresh: " + std::to_string(activeThreshold), cv::Point(20, 65),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 0), 2);
    }

    if (!rightFrame.empty()) {
      cv::cvtColor(rightFrame, displayRight, cv::COLOR_GRAY2BGR);
      if (currentMode == VIEW_GLINT_HIGHLIGHT || currentMode == VIEW_THRESHOLD_MASK) {
        cv::Mat mask;
        cv::threshold(rightFrame, mask, activeThreshold, 255, cv::THRESH_BINARY);
        if (currentMode == VIEW_THRESHOLD_MASK) {
          cv::cvtColor(mask, displayRight, cv::COLOR_GRAY2BGR);
        } else {
          // Highlight glints in bright red
          std::vector<std::vector<cv::Point>> contours;
          cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
          for (const auto& c : contours) {
            cv::Rect r = cv::boundingRect(c);
            cv::rectangle(displayRight, r, cv::Scalar(0, 0, 255), 2);
            cv::circle(displayRight, cv::Point(r.x + r.width/2, r.y + r.height/2), r.width/2 + 4, cv::Scalar(0, 255, 255), 2);
          }
        }
      }
      cv::putText(displayRight, "RIGHT | FPS: " + std::to_string(fps).substr(0, 4), cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
      cv::putText(displayRight, "Mean: " + std::to_string(meanRight[0]).substr(0, 4), cv::Point(20, 65),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 0), 2);
    }

    cv::Mat frameToDraw;
    if (currentMode == VIEW_BOTH || currentMode == VIEW_GLINT_HIGHLIGHT || currentMode == VIEW_THRESHOLD_MASK) {
      if (!displayLeft.empty() && !displayRight.empty()) {
        cv::Mat resizedL, resizedR;
        cv::resize(displayLeft, resizedL, cv::Size(640, 400));
        cv::resize(displayRight, resizedR, cv::Size(640, 400));
        cv::hconcat(resizedL, resizedR, frameToDraw);
      } else if (!displayLeft.empty()) {
        frameToDraw = displayLeft;
      } else if (!displayRight.empty()) {
        frameToDraw = displayRight;
      }
    } else if (currentMode == VIEW_LEFT_ONLY) {
      frameToDraw = displayLeft;
    } else if (currentMode == VIEW_RIGHT_ONLY) {
      frameToDraw = displayRight;
    }

    if (!frameToDraw.empty()) {
      cv::imshow(windowName, frameToDraw);
    }

    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') {
      break;
    } else if (key == '1') {
      std::cout << "[IR Strobe Debugger] Turning IR Illumination CONTINUOUSLY ON ('1')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeString("1");
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == '0') {
      std::cout << "[IR Strobe Debugger] Turning IR Illumination OFF ('0')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeString("0");
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == 'e' || key == 'E') {
      expPresetIdx = (expPresetIdx + 1) % numExpPresets;
      int us = expPresetsUs[expPresetIdx];
      std::cout << "[IR Strobe Debugger] Setting exposure to " << us << " us..." << std::endl;
      if (nodeL) nodeL->setExposure(us);
      if (nodeR) nodeR->setExposure(us);
    } else if (key == 9 || key == 'v' || key == 'V') {
      currentMode = (currentMode + 1) % 5;
    } else if (key == 's' || key == 'S' || key == 'f' || key == 'F') {
      std::cout << "[IR Strobe Debugger] Sending 300ms IR Strobe Burst ('F')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeString("F");
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == '+' || key == '=') {
      activeThreshold = (std::min)(254, activeThreshold + 5);
      std::cout << "[IR Strobe Debugger] Threshold set to: " << activeThreshold << std::endl;
    } else if (key == '-' || key == '_') {
      activeThreshold = (std::max)(10, activeThreshold - 5);
      std::cout << "[IR Strobe Debugger] Threshold set to: " << activeThreshold << std::endl;
    }
  }

  cv::destroyAllWindows();
  cameraSystem.shutdown();
  std::cout << "Strobe Debugger shutdown cleanly." << std::endl;
#else
  (void)leftCamIdx;
  (void)rightCamIdx;
  (void)comPort;
  std::cerr << "Live Camera & IR Strobe Debug Viewer is only supported on Windows (requires MediaFoundation & Win32Serial)." << std::endl;
#endif
}

void runReplayViewer(const std::string &replayDir) {
  namespace fs = std::filesystem;
  fs::path dir(replayDir);
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    std::cerr << "Error: Replay directory does not exist: " << replayDir
              << std::endl;
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
    if (!in.is_open())
      throw std::runtime_error("Could not open file");
    in >> meta;
  } catch (const std::exception &e) {
    std::cerr << "Error parsing metadata.json: " << e.what() << std::endl;
    return;
  }

  std::string shotId = meta.value("shotId", "unknown");
  std::cout << "============================================" << std::endl;
  std::cout << "Loading Shot Replay: " << shotId << std::endl;
  if (meta.contains("kinematics")) {
    auto k = meta["kinematics"];
    std::cout << "Kinematics Solved:" << std::endl;
    std::cout << "  - Speed: " << k.value("ballSpeed_mph", 0.0) << " mph"
              << std::endl;
    std::cout << "  - VLA: " << k.value("verticalLaunchAngle_deg", 0.0)
              << " deg" << std::endl;
    std::cout << "  - HLA: " << k.value("horizontalLaunchAngle_deg", 0.0)
              << " deg" << std::endl;
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
      rightImg =
          cv::imread((dir / "annotated" / ("right_" + filename)).string());
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

    std::string statusText = "Frame " + std::to_string(currentIndex + 1) +
                             " / " + std::to_string(frameCount) + " | " +
                             (playing ? "PLAYING" : "PAUSED") +
                             " | Overlays: " + (showOverlays ? "ON" : "OFF");
    cv::rectangle(composite, cv::Rect(5, 5, 450, 30), cv::Scalar(0, 0, 0), -1);
    cv::putText(composite, statusText, cv::Point(15, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    cv::imshow(windowName, composite);

    int waitTime = playing ? 100 : 0;
    int key = cv::waitKey(waitTime);

    if (key == 27) { // ESC
      break;
    } else if (key == ' ') {
      playing = !playing;
    } else if (key == 'o' || key == 'O') {
      showOverlays = !showOverlays;
    } else if (key == 'd' || key == 'D' || key == 2424832 || key == 65363 ||
               key == 79) { // right
      currentIndex = (currentIndex + 1) % frameCount;
    } else if (key == 'a' || key == 'A' || key == 2424830 || key == 65361 ||
               key == 80) { // left
      currentIndex = (currentIndex + frameCount - 1) % frameCount;
    } else if (playing) {
      currentIndex = (currentIndex + 1) % frameCount;
    }
  }

  cv::destroyWindow(windowName);
}
