#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>
#include <thread>

#include "App/AppConfig.hpp"
#include "Camera/CameraConfig.hpp"
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
#include "HAL/SerialPort.hpp"
#ifdef _WIN32
#include "HAL/MediaFoundationDriver.hpp"
#include "HAL/Win32Serial.hpp"
using PlatformCameraDriver = MediaFoundationDriver;
#else
#include "HAL/V4L2Driver.hpp"
using PlatformCameraDriver = V4L2Driver;
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
#include "Math/DotClusterFinder.hpp"
#include "Math/DotClusterTracker.hpp"
#include "Math/StereoBallTrackerTrigger.hpp"
#include "Math/StereoTriangulator.hpp"
#include "Math/TcpJsonTransmitter.hpp"
#include "Orchestration/SessionStateMachine.hpp"
#include "Orchestration/PipelineTimingConfig.hpp"
#include "Orchestration/ThreadManager.hpp"
#include <filesystem>
#include <fstream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sstream>

const bool RUN_DEBUG_VIEWER = false;

// Construct the platform camera driver by logical index, or by explicit device
// path when one was given on the command line (Linux only; ignored on Windows).
// The CameraConfig is applied by the driver during initialize().
static std::unique_ptr<PlatformCameraDriver> makeCameraDriver(int index, const std::string& devicePath,
                                                              const CameraConfig& camera) {
#ifdef __linux__
  if (!devicePath.empty()) {
    return std::make_unique<PlatformCameraDriver>(devicePath, camera);
  }
#else
  (void)devicePath;
#endif
  return std::make_unique<PlatformCameraDriver>(static_cast<uint32_t>(index), camera);
}

// What the hardware actually settled on, for the startup log and metadata.json.
static nlohmann::json describeCamera(const OV9281CameraNode& node) {
  return {
      {"appliedExposureUs", node.getAppliedExposureUs()},
      {"appliedGain", node.getAppliedGain()},
      {"negotiatedFps", node.getNegotiatedFps()},
  };
}

void runCameraDebugViewer(const AppConfig& config, int leftCamIdx, int rightCamIdx,
                          const std::string& comPort, const std::string& leftDev = "",
                          const std::string& rightDev = "");

void runReplayViewer(const std::string &replayDir);

int main(int argc, char *argv[]) {
  bool streamMode = false;
  int streamFrames = 50;
  int leftCamIdx = 1;
  int rightCamIdx = 0;
#ifdef _WIN32
  std::string comPort = "COM3";
#else
  std::string comPort = "/dev/ttyACM0";
#endif

  bool liveMode = false;
  std::string leftDev;   // Linux: explicit /dev/videoN override for --left-cam
  std::string rightDev;  // Linux: explicit /dev/videoN override for --right-cam

  // Configuration: compiled defaults <- config file <- command line
  std::string configPath = AppConfig::kDefaultPath;
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config") configPath = argv[i + 1];
  }
  AppConfig config = AppConfig::loadFromFile(configPath);
  bool swapCameras = config.stereo.swapCameras;
  bool ignoreTiming = false;

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "GolfSim [--live] [--stream [--frames N]] [--replay DIR]\n"
                << "        [--config PATH] [--exposure-us N] [--gain N] [--fps N] [--intensity N]\n"
                << "        [--left-cam N] [--right-cam N] [--left-dev PATH] [--right-dev PATH]\n"
                << "        [--swap-cameras] [--com PORT] [--ignore-timing] [--version]\n"
                << "Config file (default " << AppConfig::kDefaultPath << ") holds camera, detector and stereo settings;\n"
                << "command-line values override it." << std::endl;
      return 0;
    }
    if (arg == "--version" || arg == "-v") {
      std::cout << "GolfSim"
                << "  OpenCV " << CV_VERSION
                << "  Eigen " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION
                << "  spdlog " << SPDLOG_VER_MAJOR << "." << SPDLOG_VER_MINOR << "." << SPDLOG_VER_PATCH
                << "  json " << NLOHMANN_JSON_VERSION_MAJOR << "." << NLOHMANN_JSON_VERSION_MINOR << "." << NLOHMANN_JSON_VERSION_PATCH
                << std::endl;
      return 0;
    }
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
    if (arg == "--left-dev" && i + 1 < argc) {
      leftDev = argv[++i];
    }
    if (arg == "--right-dev" && i + 1 < argc) {
      rightDev = argv[++i];
    }
    if (arg == "--swap-cameras" || arg == "--swap") {
      swapCameras = true;
    }
    if (arg == "--com" && i + 1 < argc) {
      comPort = argv[++i];
    }
    if (arg == "--config" && i + 1 < argc) {
      ++i;  // consumed above
    }
    if (arg == "--exposure-us" && i + 1 < argc) {
      config.camera.exposureUs = std::atoi(argv[++i]);
    }
    if (arg == "--gain" && i + 1 < argc) {
      config.camera.gain = std::atoi(argv[++i]);
    }
    if (arg == "--fps" && i + 1 < argc) {
      config.camera.targetFps = std::atoi(argv[++i]);
    }
    if (arg == "--intensity" && i + 1 < argc) {
      config.detector.intensityThreshold = std::atoi(argv[++i]);
    }
    if (arg == "--ignore-timing") {
      ignoreTiming = true;
    }
  }
  config.camera.clampToHardwareRanges();
  config.stereo.swapCameras = swapCameras;
  if (swapCameras) {
    std::swap(leftCamIdx, rightCamIdx);
    std::swap(leftDev, rightDev);
  }

  if (liveMode || RUN_DEBUG_VIEWER) {
    spdlog::info("[Config] {}", config.describe());
    runCameraDebugViewer(config, leftCamIdx, rightCamIdx, comPort, leftDev, rightDev);
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

  spdlog::info("[Config] {}", config.describe());
  spdlog::info("[System] Initializing camera drivers...");
  PlatformCameraDriver::logConnectedDevices();

  // 2. Initialize the platform camera drivers with correct hardware device mapping
  spdlog::info("[System] Mapping Left Camera to Index {}{}, Right Camera to Index {}{}",
               leftCamIdx, leftDev.empty() ? "" : " (" + leftDev + ")",
               rightCamIdx, rightDev.empty() ? "" : " (" + rightDev + ")");
  auto usbLeft = makeCameraDriver(leftCamIdx, leftDev, config.camera);
  bool leftOk = usbLeft->initialize();

  auto usbRight = makeCameraDriver(rightCamIdx, rightDev, config.camera);
  bool rightOk = usbRight->initialize();

  if (!leftOk && !rightOk) {
    spdlog::warn("[System] Failed to initialize camera hardware (expected in "
                 "emulation/test environments). Clean exit.");
    return 0;
  }

  uint32_t width = 1280;
  uint32_t height = 800;
  nlohmann::json sessionInfo = {
      {"config", config.toJson()},
      {"configSource", config.sourcePath.empty() ? "compiled defaults" : config.sourcePath},
  };
  int appliedExposureUs = CameraConfig::quantiseExposureUs(config.camera.exposureUs);
  double negotiatedFps = 0.0;

  if (leftOk) {
    width = usbLeft->getFrameWidth();
    height = usbLeft->getFrameHeight();
    auto cameraNodeLeft = std::make_shared<OV9281CameraNode>(
        std::move(usbLeft), CameraRole::STEREO_LEFT);
    cameraSystem->addCameraNode(cameraNodeLeft);
    sessionInfo["left"] = describeCamera(*cameraNodeLeft);
    if (cameraNodeLeft->getAppliedExposureUs() > 0) appliedExposureUs = cameraNodeLeft->getAppliedExposureUs();
    if (cameraNodeLeft->getNegotiatedFps() > 0.0) negotiatedFps = cameraNodeLeft->getNegotiatedFps();
    spdlog::info("[System] Registered Left camera ({}x{}): exposure {} us, gain {}, {:.1f} fps",
                 width, height, cameraNodeLeft->getAppliedExposureUs(),
                 cameraNodeLeft->getAppliedGain(), cameraNodeLeft->getNegotiatedFps());
  }

  if (rightOk) {
    width = usbRight->getFrameWidth();
    height = usbRight->getFrameHeight();
    auto cameraNodeRight = std::make_shared<OV9281CameraNode>(
        std::move(usbRight), CameraRole::STEREO_RIGHT);
    cameraSystem->addCameraNode(cameraNodeRight);
    sessionInfo["right"] = describeCamera(*cameraNodeRight);
    if (appliedExposureUs <= 0 && cameraNodeRight->getAppliedExposureUs() > 0) {
      appliedExposureUs = cameraNodeRight->getAppliedExposureUs();
    }
    if (negotiatedFps <= 0.0 && cameraNodeRight->getNegotiatedFps() > 0.0) {
      negotiatedFps = cameraNodeRight->getNegotiatedFps();
    }
    spdlog::info("[System] Registered Right camera ({}x{}): exposure {} us, gain {}, {:.1f} fps",
                 width, height, cameraNodeRight->getAppliedExposureUs(),
                 cameraNodeRight->getAppliedGain(), cameraNodeRight->getNegotiatedFps());
  }

  // Queue buffer manager (capacity of 16 FrameSets)
  auto buffer = std::make_shared<AtomicRingBuffer<FrameSet, 16>>();
  buffer->preallocate(1280, 800);

  // Pipeline components initialization:
  // - Left camera optical gate trigger (Region of Interest, min Ball pixels,
  // diff threshold, EMA alpha)
  // - Image Moments tracker (Ball threshold, Marker threshold, min Area, max
  // Area, min Circularity)
  // - Stereo Triangulator (Uses default horizontal calibration)
  // - Kinematics physics engine
  // - Local network TCP transmitter (Target loopback, port 9002)
  PipelineTimingConfig timingConfig;
  timingConfig.workingDistanceMeters = 0.9144; // 3.0 ft
  timingConfig.nominalBallRadiusPx   = 23.3;   // ~23.3 px radius at 3.0 ft
  timingConfig.minPointsToSolve      = 3;      // Minimum 3 points
  timingConfig.maxFramesPerShot      = 2;      // 2 frames maximum for irons/wedges
  timingConfig.emptyFrameTimeout     = 1;      // Solve immediately on 1st empty frame
  timingConfig.cameraExposureUs      = appliedExposureUs;
  timingConfig.cameraFrameRateHz     = negotiatedFps > 0.0 ? negotiatedFps
                                                            : static_cast<double>(config.camera.targetFps);

  // The strobe train must fit inside the exposure, and the exposure inside one
  // frame period — otherwise pulses are silently lost. Refuse rather than guess.
  if (!timingConfig.isValidTiming()) {
    spdlog::error("[System] Invalid strobe/exposure timing: exposure {} us must hold the {:.0f} us "
                  "{}-pulse train at {:.0f} Hz and stay under the {:.0f} us frame period at {:.1f} fps. "
                  "Fix config/golfsim.json (camera.exposureUs) or pass --ignore-timing to run anyway.",
                  timingConfig.cameraExposureUs, timingConfig.strobeTrainDurationUs(),
                  timingConfig.strobePulseCount, timingConfig.highStrobeRateHz,
                  timingConfig.framePeriodUs(), timingConfig.cameraFrameRateHz);
    if (!ignoreTiming) return 1;
    spdlog::warn("[System] --ignore-timing set; continuing with invalid timing.");
  }

  // Dots-only detection shared by the trigger and the vision stage; stereo
  // pairing gates from config, whole-frame search (no ROI).
  auto trigger = StereoBallTrackerTrigger(
      StereoCalibration(), config.detector, timingConfig.nominalBallRadiusPx,
      config.stereo.epipolarTolerancePx, config.stereo.disparityMinPx,
      config.stereo.disparityMaxPx, 256, 0.9144, 5, 4, 4.0, 0.04);
  auto vision = DotClusterTracker(config.detector, timingConfig.nominalBallRadiusPx);
  auto spatial = StereoTriangulator();
  auto kinematics = EigenBallisticsEngine();
  auto network = TcpJsonTransmitter("127.0.0.1", 9002);

  auto stateMachine = std::make_shared<ConcreteSSM>(trigger, vision, spatial,
                                                    kinematics, network, timingConfig);
  sessionInfo["timing"] = {
      {"cameraExposureUs", timingConfig.cameraExposureUs},
      {"cameraFrameRateHz", timingConfig.cameraFrameRateHz},
      {"strobePulseCount", timingConfig.strobePulseCount},
      {"pulseIntervalMs", timingConfig.pulseIntervalMs},
  };
  stateMachine->setSessionInfo(sessionInfo);

  // Initialize Serial connection to Arduino Strobe Controller (with 2 retries before graceful degradation)
  SerialPort serial;
  if (serial.openWithRetry(comPort, 115200, 2, 500)) {
    spdlog::info("[System] Connected to IR Strobe Controller on {}", comPort);
  } else {
    spdlog::warn("[System] Could not connect to IR Strobe Controller on {}. Running in offline/simulation mode.", comPort);
  }

  // Connect serial callback from SessionStateMachine
  stateMachine->setSerialCallback([&serial](char cmd) {
    if (serial.isOpen()) {
      serial.writeChar(cmd);
    }
  });

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

  if (serial.isOpen()) {
    spdlog::info("[System] Powering off IR emitters ('0')...");
    serial.writeChar('0');
    serial.close();
  }

  spdlog::info("[System] Shutdown completed cleanly.");
  return 0;
}

// -------------------------------------------------------------------------
// Live Camera Setup and Test Viewer (RUN_DEBUG_VIEWER = true)
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// Live Camera Setup and IR Strobe Debug Viewer
// -------------------------------------------------------------------------
void runCameraDebugViewer(const AppConfig& config, int leftCamIdx, int rightCamIdx,
                          const std::string& comPort, const std::string& leftDev,
                          const std::string& rightDev) {
  std::cout << "\n============================================" << std::endl;
  std::cout << "Starting Live Camera & IR Strobe Debug Viewer" << std::endl;
  std::cout << "============================================" << std::endl;

  SerialPort serial;
  if (serial.openWithRetry(comPort, 115200, 2, 500)) {
      std::cout << "Successfully connected to Arduino on " << comPort << std::endl;
  } else {
      std::cerr << "Warning: Could not open Serial port " << comPort << ". Hardware strobe testing disabled." << std::endl;
  }

  // 1. Enumerate connected video capture devices
  PlatformCameraDriver::logConnectedDevices();

  // 2. Initialize platform camera drivers with user-specified indices
  std::cout << "\nInitializing Left camera (Index " << leftCamIdx
            << (leftDev.empty() ? "" : ", " + leftDev) << ")..." << std::endl;
  auto usbDriverLeft = makeCameraDriver(leftCamIdx, leftDev, config.camera);
  bool leftOk = usbDriverLeft->initialize();

  std::cout << "Initializing Right camera (Index " << rightCamIdx
            << (rightDev.empty() ? "" : ", " + rightDev) << ")..." << std::endl;
  auto usbDriverRight = makeCameraDriver(rightCamIdx, rightDev, config.camera);
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
    std::cout << "Successfully initialized Left camera (" << width << "x" << height << ")" << std::endl;
  }

  if (rightOk) {
    width = usbDriverRight->getFrameWidth();
    height = usbDriverRight->getFrameHeight();
    nodeR = std::make_shared<OV9281CameraNode>(
        std::move(usbDriverRight), CameraRole::STEREO_RIGHT);
    cameraSystem.addCameraNode(nodeR);
    std::cout << "Successfully initialized Right camera (" << width << "x" << height << ")" << std::endl;
  }

  FrameSet frameSet;
  frameSet.preallocate(width, height);

  enum ViewMode { VIEW_BOTH, VIEW_LEFT_ONLY, VIEW_RIGHT_ONLY, VIEW_GLINT_HIGHLIGHT, VIEW_THRESHOLD_MASK };
  int currentMode = VIEW_BOTH;

  // Live detection with the production detector so the viewer shows exactly
  // what the pipeline will see. '+'/'-' move the intensity threshold.
  DotClusterConfig dotConfig = config.detector;
  int activeThreshold = dotConfig.intensityThreshold;
  DotClusterFinder finder(dotConfig, 23.3);
  DotClusterResult dotsL, dotsR;

  // Exposure moves in the hardware's own log2 steps; gain in steps of 5.
  int exposureLog2 = CameraConfig::exposureUsToLog2(config.camera.exposureUs);
  int gain = config.camera.gain;
  auto appliedExposureUs = [&]() {
    int us = nodeL ? nodeL->getAppliedExposureUs() : -1;
    if (us <= 0 && nodeR) us = nodeR->getAppliedExposureUs();
    return us > 0 ? us : CameraConfig::exposureLog2ToUs(exposureLog2);
  };
  auto negotiatedFps = [&]() {
    double f = nodeL ? nodeL->getNegotiatedFps() : 0.0;
    if (f <= 0.0 && nodeR) f = nodeR->getNegotiatedFps();
    return f;
  };
  std::string strobeStatusStr = "STROBE: 300 Hz READY";

  if (serial.isOpen()) {
      serial.writeChar('H');
  }

  std::cout << "\n=======================================================" << std::endl;
  std::cout << "GOLFSIM 300 HZ IR STROBE DEBUGGER CONTROLS:" << std::endl;
  std::cout << "  - Press 'h' / 'H' : 300 Hz Strobe Active Mode ('H')" << std::endl;
  std::cout << "  - Press 'l' / 'L' : 10 Hz Standby Protection Mode ('L')" << std::endl;
  std::cout << "  - Press '1'       : Turn IR Illumination CONTINUOUSLY ON (Aiming)" << std::endl;
  std::cout << "  - Press '0'       : Turn IR Illumination OFF ('0')" << std::endl;
  std::cout << "  - Press 's' / 'f' : Fire Single 3-Pulse 300 Hz Test Burst ('F')" << std::endl;
  std::cout << "  - Press 'e' / 'E' : Exposure one UVC step down / up (122us ... 31ms, x2 per step)" << std::endl;
  std::cout << "  - Press 'g' / 'G' : Gain -5 / +5 (0..100)" << std::endl;
  std::cout << "  - Press 'v'       : Cycle View Modes (Both -> Left -> Right -> Dot Clusters -> Threshold Mask)" << std::endl;
  std::cout << "  - Press '+' / '-' : Adjust Intensity Threshold (Current: " << activeThreshold << ")" << std::endl;
  std::cout << "  - Press 'p'       : Print the current camera/detector values as config JSON" << std::endl;
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
    char expBuf[96];
    snprintf(expBuf, sizeof(expBuf), "Exp: %dus | Gain: %d | Cam: %.0f fps", appliedExposureUs(),
             gain, negotiatedFps());
    std::string expStr = expBuf;

    // Draw what the production detector sees: dots (blue), accepted clusters
    // (green box + count), rejected dots/clusters (red).
    auto drawDots = [&](cv::Mat& canvas, const DotClusterResult& res) {
      for (const auto& r : res.rejectedDots) {
        cv::rectangle(canvas, r.dot.box, cv::Scalar(0, 0, 255), 1);
      }
      for (const auto& r : res.rejectedClusters) {
        cv::rectangle(canvas, r.cluster.boundingBox, cv::Scalar(0, 0, 255), 1);
        cv::putText(canvas, r.reason, cv::Point(r.cluster.boundingBox.x, r.cluster.boundingBox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
      }
      for (const auto& c : res.clusters) {
        for (const auto& d : c.dots) {
          cv::circle(canvas, d.centroid, 4, cv::Scalar(255, 0, 0), 1);
        }
        cv::rectangle(canvas, c.boundingBox, cv::Scalar(0, 255, 0), 2);
        cv::drawMarker(canvas, c.centroid, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 12, 1);
        char lbl[64];
        snprintf(lbl, sizeof(lbl), "Ball: %zu dots, %.0f px", c.dots.size(), c.spreadPx);
        cv::putText(canvas, lbl, cv::Point(c.boundingBox.x, c.boundingBox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1);
      }
    };

    if (!leftFrame.empty()) {
      cv::cvtColor(leftFrame, displayLeft, cv::COLOR_GRAY2BGR);
      if (currentMode == VIEW_THRESHOLD_MASK) {
        cv::Mat mask;
        cv::threshold(leftFrame, mask, activeThreshold, 255, cv::THRESH_BINARY);
        cv::cvtColor(mask, displayLeft, cv::COLOR_GRAY2BGR);
      } else if (currentMode == VIEW_GLINT_HIGHLIGHT) {
        finder.find(leftFrame, dotsL);
        drawDots(displayLeft, dotsL);
      }
      cv::putText(displayLeft, "LEFT | " + strobeStatusStr, cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, (strobeStatusStr.find("OFF") == std::string::npos) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
      cv::putText(displayLeft, expStr + " | Thresh: " + std::to_string(activeThreshold), cv::Point(20, 65),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 0), 2);
      if (currentMode == VIEW_GLINT_HIGHLIGHT) {
        char dotBuf[96];
        snprintf(dotBuf, sizeof(dotBuf), "Clusters: %zu | Dots rejected: %zu", dotsL.clusters.size(),
                 dotsL.rejectedDots.size());
        cv::putText(displayLeft, dotBuf, cv::Point(20, 95), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 2);
      }
    }

    if (!rightFrame.empty()) {
      cv::cvtColor(rightFrame, displayRight, cv::COLOR_GRAY2BGR);
      if (currentMode == VIEW_THRESHOLD_MASK) {
        cv::Mat mask;
        cv::threshold(rightFrame, mask, activeThreshold, 255, cv::THRESH_BINARY);
        cv::cvtColor(mask, displayRight, cv::COLOR_GRAY2BGR);
      } else if (currentMode == VIEW_GLINT_HIGHLIGHT) {
        finder.find(rightFrame, dotsR);
        drawDots(displayRight, dotsR);
      }
      cv::putText(displayRight, "RIGHT | Measured FPS: " + std::to_string(fps).substr(0, 4), cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 0), 2);
      cv::putText(displayRight, "Mean: " + std::to_string(meanRight[0]).substr(0, 4) +
                  "  Mean L: " + std::to_string(meanLeft[0]).substr(0, 4), cv::Point(20, 65),
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 0), 2);
      if (currentMode == VIEW_GLINT_HIGHLIGHT) {
        char dotBuf[128];
        snprintf(dotBuf, sizeof(dotBuf), "Clusters: %zu | Dots rejected: %zu", dotsR.clusters.size(),
                 dotsR.rejectedDots.size());
        if (!dotsL.clusters.empty() && !dotsR.clusters.empty()) {
          const auto& l = dotsL.clusters[0].centroid;
          const auto& r = dotsR.clusters[0].centroid;
          snprintf(dotBuf + strlen(dotBuf), sizeof(dotBuf) - strlen(dotBuf), " | xL-xR %.0f  yL-yR %.0f",
                   l.x - r.x, l.y - r.y);
        }
        cv::putText(displayRight, dotBuf, cv::Point(20, 95), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 2);
      }
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
    } else if (key == 'h' || key == 'H') {
      std::cout << "[IR Strobe Debugger] Activating 300 Hz Strobe Mode ('H')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeChar('H');
          strobeStatusStr = "STROBE: 300 Hz READY";
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == 'l' || key == 'L') {
      std::cout << "[IR Strobe Debugger] Activating 10 Hz Standby Protection Mode ('L')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeChar('L');
          strobeStatusStr = "STROBE: 10 Hz STANDBY";
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == '1') {
      std::cout << "[IR Strobe Debugger] Turning IR Illumination CONTINUOUSLY ON ('1')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeChar('1');
          strobeStatusStr = "STROBE: DC ON (Aiming)";
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == '0') {
      std::cout << "[IR Strobe Debugger] Turning IR Illumination OFF ('0')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeChar('0');
          strobeStatusStr = "STROBE: OFF";
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == 'e' || key == 'E') {
      exposureLog2 += (key == 'E') ? 1 : -1;
      exposureLog2 = std::clamp(exposureLog2, CameraConfig::kMinExposureLog2, CameraConfig::kMaxExposureLog2);
      int us = CameraConfig::exposureLog2ToUs(exposureLog2);
      std::cout << "[IR Strobe Debugger] Setting exposure to " << us << " us (log2 " << exposureLog2 << ")..." << std::endl;
      if (nodeL) nodeL->setExposure(us);
      if (nodeR) nodeR->setExposure(us);
    } else if (key == 'g' || key == 'G') {
      gain = std::clamp(gain + ((key == 'G') ? 5 : -5), 0, CameraConfig::kMaxGain);
      std::cout << "[IR Strobe Debugger] Setting gain to " << gain << "..." << std::endl;
      if (nodeL) nodeL->setGain(gain);
      if (nodeR) nodeR->setGain(gain);
    } else if (key == 'p' || key == 'P') {
      CameraConfig cam = config.camera;
      cam.exposureUs = appliedExposureUs();
      cam.gain = gain;
      DotClusterConfig det = dotConfig;
      det.intensityThreshold = activeThreshold;
      nlohmann::json snapshot = {{"camera", cam.toJson()}, {"detector", det.toJson()}};
      std::cout << "[IR Strobe Debugger] Current settings (paste into config/golfsim.json):\n"
                << snapshot.dump(2) << std::endl;
    } else if (key == 9 || key == 'v' || key == 'V') {
      currentMode = (currentMode + 1) % 5;
    } else if (key == 's' || key == 'S' || key == 'f' || key == 'F') {
      std::cout << "[IR Strobe Debugger] Sending Single 300 Hz Test Burst ('F')..." << std::endl;
      if (serial.isOpen()) {
          serial.writeChar('F');
      } else {
          std::cout << "[IR Strobe Debugger] Serial not connected." << std::endl;
      }
    } else if (key == '+' || key == '=') {
      activeThreshold = (std::min)(254, activeThreshold + 5);
      dotConfig.intensityThreshold = activeThreshold;
      finder.setConfig(dotConfig);
      std::cout << "[IR Strobe Debugger] Intensity threshold set to: " << activeThreshold << std::endl;
    } else if (key == '-' || key == '_') {
      activeThreshold = (std::max)(10, activeThreshold - 5);
      dotConfig.intensityThreshold = activeThreshold;
      finder.setConfig(dotConfig);
      std::cout << "[IR Strobe Debugger] Intensity threshold set to: " << activeThreshold << std::endl;
    }
  }

  if (serial.isOpen()) {
    serial.writeChar('0');
    serial.close();
  }

  cv::destroyAllWindows();
  cameraSystem.shutdown();
  std::cout << "Strobe Debugger shutdown cleanly." << std::endl;
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
