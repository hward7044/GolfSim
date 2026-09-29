#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
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

// The pipeline's timing/geometry for this rig, with the exposure and frame
// rate the hardware actually settled on. Production validates against this;
// the debug viewer confines its exposure steps to it, so the two agree.
static PipelineTimingConfig makePipelineTiming(int appliedExposureUs, double frameRateHz,
                                               const StrobeConfig& strobe) {
  PipelineTimingConfig timing;
  timing.workingDistanceMeters = 0.9144; // 3.0 ft
  timing.nominalBallRadiusPx   = 23.3;   // ~23.3 px radius at 3.0 ft
  timing.minPointsToSolve      = 3;      // Minimum 3 points
  timing.maxFramesPerShot      = 2;      // 2 frames maximum for irons/wedges
  timing.emptyFrameTimeout     = 1;      // Solve immediately on 1st empty frame
  timing.cameraExposureUs      = appliedExposureUs;
  timing.cameraFrameRateHz     = frameRateHz;
  timing.subPulseDurationUs    = strobe.pulseWidthUs;
  timing.setStrobeRateHz(StrobeConfig::kRateHz);
  return timing;
}

// Program the controller's pulse width ("W<us>" + newline). It clamps to the
// same envelope as StrobeConfig and echoes what it applied.
static void sendStrobePulseWidth(SerialPort& serial, int pulseWidthUs) {
  if (serial.isOpen()) serial.writeString("W" + std::to_string(pulseWidthUs) + "\n");
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

  // Every relative path (config file, session log, replays, shot history) is
  // anchored on the project root, so the binary behaves the same whether it
  // is launched from the repo root, from build/, or from an IDE.
  const std::string projectRoot = AppConfig::findProjectRoot();
  auto rootedPath = [&projectRoot](const char* rel) {
    return projectRoot.empty() ? std::string(rel)
                               : (std::filesystem::path(projectRoot) / rel).make_preferred().string();
  };

  // Configuration: compiled defaults <- config file <- command line
  std::string configPath = rootedPath(AppConfig::kDefaultPath);
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config") configPath = argv[i + 1];
  }
  AppConfig config = AppConfig::loadFromFile(configPath);
  bool swapCameras = config.stereo.swapCameras;
  bool ignoreTiming = false;

  // Silent fallback to compiled defaults is how a tuned config goes unnoticed;
  // say so wherever the config summary is logged.
  auto logConfig = [&config]() {
    spdlog::info("[Config] {}", config.describe());
    if (config.sourcePath.empty()) {
      spdlog::warn("[Config] Running on compiled defaults: no {} found above the working "
                   "directory or the executable, and no --config given.",
                   AppConfig::kDefaultPath);
    }
  };

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "GolfSim [--live] [--stream [--frames N]] [--replay DIR]\n"
                << "        [--config PATH] [--exposure-us N] [--gain N] [--fps N] [--intensity N]\n"
                << "        [--left-cam N] [--right-cam N] [--left-dev PATH] [--right-dev PATH]\n"
                << "        [--swap-cameras] [--com PORT] [--ignore-timing] [--version]\n"
                << "Config file (default " << AppConfig::kDefaultPath << ", resolved against the nearest\n"
                << "project root above the working directory or the executable) holds camera,\n"
                << "detector and stereo settings; command-line values override it." << std::endl;
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
    logConfig();
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
        rootedPath("build/session.log"), true);
    spdlog::set_default_logger(std::make_shared<spdlog::logger>(
        "multi_sink", spdlog::sinks_init_list({console_sink, file_sink})));
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_on(spdlog::level::info);
  } catch (const spdlog::spdlog_ex &ex) {
    std::cerr << "Log initialization failed: " << ex.what() << std::endl;
  }

  auto cameraSystem = std::make_shared<HardwareSyncedCameraSystem>();

  logConfig();
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
  PipelineTimingConfig timingConfig = makePipelineTiming(
      appliedExposureUs,
      negotiatedFps > 0.0 ? negotiatedFps : static_cast<double>(config.camera.targetFps),
      config.strobe);

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
                                                    kinematics, network, timingConfig,
                                                    rootedPath("build/replays"),
                                                    rootedPath("build/shot_history.json"));
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
    spdlog::info("[System] Connected to IR Strobe Controller on {}; pulse width {} us ({:.1f}% duty at {} Hz)",
                 comPort, config.strobe.pulseWidthUs, config.strobe.dutyCycle() * 100.0, StrobeConfig::kRateHz);
    sendStrobePulseWidth(serial, config.strobe.pulseWidthUs);
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
  auto applyExposure = [&](int us) {
    if (nodeL) nodeL->setExposure(us);
    if (nodeR) nodeR->setExposure(us);
  };

  // Only exposures the pipeline will accept can be selected here: the strobe
  // train must fit inside the exposure, and the exposure inside one frame at
  // the rate the cameras actually negotiated. Otherwise a value tuned in this
  // viewer is refused at production startup.
  const double frameRateHz = negotiatedFps() > 0.0 ? negotiatedFps()
                                                   : static_cast<double>(config.camera.targetFps);
  StrobeConfig strobe = config.strobe;   // 'w'/'W' step the pulse width inside its envelope
  // Bench mode ('b'): stationary brightness tests past the 100 us blur cap, up
  // to the firmware's optical budget. A pulse of width w at today's current
  // has the same energy (and shot noise) as a w/5 pulse at 5x current, so it
  // previews the overdrive plan without touching the board.
  bool   benchMode    = false;
  double mcuOverdrive = 1.0;             // LED_OVERDRIVE_RATIO the controller reports on 'P'
  auto pulseWidthLimitUs = [&]() {
    return benchMode ? StrobeConfig::benchPulseWidthLimitUs(mcuOverdrive) : StrobeConfig::kMaxPulseWidthUs;
  };
  PipelineTimingConfig timing = makePipelineTiming(appliedExposureUs(), frameRateHz, strobe);
  PipelineTimingConfig::ExposureLog2Range safeExposure = timing.validExposureLog2Range();
  auto safeRangeStr = [&]() {
    if (safeExposure.empty()) return std::string("none");
    return std::to_string(CameraConfig::exposureLog2ToUs(safeExposure.lo)) + ".." +
           std::to_string(CameraConfig::exposureLog2ToUs(safeExposure.hi)) + " us";
  };
  if (safeExposure.empty()) {
    std::cerr << "Warning: no UVC exposure step holds the " << static_cast<int>(timing.strobeTrainDurationUs())
              << " us strobe train inside the " << static_cast<int>(timing.framePeriodUs()) << " us frame period at "
              << frameRateHz << " fps. Exposure keys are limited to the hardware range only." << std::endl;
  } else if (!timing.isValidExposureUs(appliedExposureUs())) {
    int snapped = safeExposure.clamp(exposureLog2);
    std::cout << "Warning: configured exposure " << appliedExposureUs() << " us is outside the strobe-safe range ("
              << safeRangeStr() << "); snapping to " << CameraConfig::exposureLog2ToUs(snapped) << " us." << std::endl;
    exposureLog2 = snapped;
    applyExposure(CameraConfig::exposureLog2ToUs(exposureLog2));
  }
  // Brightest pixel seen in the last two seconds, so a 10 Hz standby pulse or
  // an 'F' burst registers even when the display loop skips that frame.
  double peakL = 0.0, peakR = 0.0;
  auto peakAt = std::chrono::steady_clock::now();
  // 'c' saves the current frame pair + settings here for offline inspection.
  const std::string projectRoot = AppConfig::findProjectRoot();
  const std::filesystem::path captureDir =
      (projectRoot.empty() ? std::filesystem::path("build") : std::filesystem::path(projectRoot) / "build") / "captures";
  std::string strobeStatusStr = "STROBE: 300 Hz READY";

  // Controller link. Every reply is echoed so a mode change that did not land
  // is visible, and a 'P' ping once a second both holds the firmware's 10 s
  // watchdog off (it drops 300 Hz mode without PC traffic) and reads back the
  // pulse counter, so the HUD shows the rate the LEDs are really being driven
  // at: 300/s in READY, 10/s in STANDBY, 0 in OFF and DC.
  std::string mcuStatusStr = serial.isOpen() ? "MCU: waiting for ping reply" : "MCU: not connected";
  std::string serialLineBuf;
  unsigned long lastPulses = 0;
  auto lastPingAt = std::chrono::steady_clock::now();
  auto lastPulsesAt = lastPingAt;
  bool havePulses = false;
  bool pulseRateMissing = false;   // a pulsed mode reporting 0/s: firmware or wiring fault
  auto pollController = [&]() {
    if (!serial.isOpen()) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - lastPingAt >= std::chrono::seconds(1)) {
      serial.writeChar('P');
      lastPingAt = now;
    }
    serialLineBuf += serial.readAvailable();
    size_t eol;
    while ((eol = serialLineBuf.find('\n')) != std::string::npos) {
      std::string line = serialLineBuf.substr(0, eol);
      serialLineBuf.erase(0, eol + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (line.empty()) continue;
      if (line.rfind("OK mode=", 0) == 0) {
        // "OK mode=READY pulses=N width=30 od=1.0" (width/od absent on older builds)
        auto field = [&line](const char* key) -> const char* {
          size_t at = line.find(key);
          return at == std::string::npos ? nullptr : line.c_str() + at + std::strlen(key);
        };
        const size_t modeAt = 8, modeEnd = line.find(' ', modeAt);
        std::string mode = line.substr(modeAt, modeEnd == std::string::npos ? std::string::npos : modeEnd - modeAt);
        const char* pf = field(" pulses=");
        unsigned long pulses = pf ? std::strtoul(pf, nullptr, 10) : 0;
        const char* w = field(" width=");
        int mcuWidthUs = w ? std::atoi(w) : -1;
        const char* od = field(" od=");
        double overdrive = od ? std::atof(od) : 1.0;
        mcuOverdrive = overdrive;
        char buf[128];
        int n;
        if (havePulses) {
          std::chrono::duration<double> dt = now - lastPulsesAt;
          double pulsesPerSec = dt.count() > 0.0 ? static_cast<double>(pulses - lastPulses) / dt.count() : 0.0;
          n = snprintf(buf, sizeof(buf), "MCU: %s | LED pulses %lu (%.0f/s)", mode.c_str(), pulses, pulsesPerSec);
          pulseRateMissing = (mode == "READY" || mode == "STANDBY") && pulsesPerSec < 1.0;
        } else {
          n = snprintf(buf, sizeof(buf), "MCU: %s | LED pulses %lu (measuring)", mode.c_str(), pulses);
        }
        // The controller may have clamped the width further than the PC did
        // (LED_OVERDRIVE_RATIO in the firmware); show what it actually runs.
        if (mcuWidthUs >= 0 && n > 0 && n < static_cast<int>(sizeof(buf))) {
          snprintf(buf + n, sizeof(buf) - n, " | %dus x%.1f%s", mcuWidthUs, overdrive,
                   mcuWidthUs != strobe.pulseWidthUs ? " (clamped by MCU)" : "");
        }
        lastPulses = pulses;
        lastPulsesAt = now;
        havePulses = true;
        mcuStatusStr = buf;
      } else if (line == "OK") {
        mcuStatusStr = "MCU: old firmware (camera-sync build, never pulses): reflash firmware/strobe_controller";
        pulseRateMissing = true;
      } else {
        std::cout << "[Arduino] " << line << std::endl;
      }
    }
  };

  if (serial.isOpen()) {
      sendStrobePulseWidth(serial, strobe.pulseWidthUs);
      serial.writeChar('H');
  }

  std::cout << "\n=======================================================" << std::endl;
  std::cout << "GOLFSIM 300 HZ IR STROBE DEBUGGER CONTROLS:" << std::endl;
  std::cout << "  (Tune exposure/threshold with the 300 Hz strobe active ('H'): that is what the pipeline runs.)" << std::endl;
  std::cout << "  - Press 'h' / 'H' : 300 Hz Strobe Active Mode ('H')" << std::endl;
  std::cout << "  - Press 'l' / 'L' : 10 Hz Standby Protection Mode ('L')" << std::endl;
  std::cout << "  - Press '1'       : Turn IR Illumination CONTINUOUSLY ON (Aiming)" << std::endl;
  std::cout << "  - Press '0'       : Turn IR Illumination OFF ('0')" << std::endl;
  std::cout << "  - Press 's' / 'f' : Fire Single 3-Pulse 300 Hz Test Burst ('F')" << std::endl;
  std::cout << "  HUD 'MCU' line: controller mode and LED pulse rate it reports (300/s in 'H', 10/s in 'L')." << std::endl;
  std::cout << "  HUD 'Max' is the brightest pixel: compare it under '1' (DC) and 'H' to size the strobe light budget." << std::endl;
  std::cout << "  - Press 'e' / 'E' : Exposure one UVC step down / up, kept strobe-safe (" << safeRangeStr()
            << " at " << frameRateHz << " fps)" << std::endl;
  std::cout << "  - Press 'g' / 'G' : Gain -5 / +5 (0..100)" << std::endl;
  std::cout << "  - Press 'w' / 'W' : Strobe pulse width -10 / +10 us (" << StrobeConfig::kMinPulseWidthUs << ".."
            << StrobeConfig::kMaxPulseWidthUs << " us; duty <= " << StrobeConfig::kMaxDutyCycle * 100.0
            << "% at " << StrobeConfig::kRateHz << " Hz). Wider = brighter per pulse, more blur in flight." << std::endl;
  std::cout << "  - Press 'b'       : BENCH mode: unlock pulse width up to the optical budget ("
            << StrobeConfig::benchPulseWidthLimitUs(1.0) << " us at 1x) for a STATIONARY ball." << std::endl;
  std::cout << "                      A w us pulse now = a w/5 us pulse at 5x current: 500 us previews 5x@100us." << std::endl;
  std::cout << "  - Press 'c'       : Save the current left/right frames + settings to " << captureDir.string() << std::endl;
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
    pollController();
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
    double maxLeft = 0.0, maxRight = 0.0;
    if (!leftFrame.empty())  cv::minMaxLoc(leftFrame,  nullptr, &maxLeft);
    if (!rightFrame.empty()) cv::minMaxLoc(rightFrame, nullptr, &maxRight);
    if (current_time - peakAt > std::chrono::seconds(2)) { peakL = 0.0; peakR = 0.0; peakAt = current_time; }
    if (maxLeft  > peakL) { peakL = maxLeft;  peakAt = current_time; }
    if (maxRight > peakR) { peakR = maxRight; peakAt = current_time; }

    cv::Mat displayLeft, displayRight;
    char expBuf[96];
    snprintf(expBuf, sizeof(expBuf), "Exp: %dus (safe %s) | Gain: %d | Pulse: %dus | Cam: %.0f fps",
             appliedExposureUs(), safeRangeStr().c_str(), gain, strobe.pulseWidthUs, negotiatedFps());
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
      cv::putText(displayLeft, mcuStatusStr, cv::Point(20, 95), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  pulseRateMissing ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0), 2);
      if (benchMode) {
        char benchBuf[160];
        snprintf(benchBuf, sizeof(benchBuf),
                 "BENCH %dus = %.1fx@100us / %.0fx@30us | blur %.0f mm @100mph | STATIONARY TEST ONLY",
                 strobe.pulseWidthUs, strobe.pulseWidthUs / 100.0, strobe.pulseWidthUs / 30.0,
                 StrobeConfig::blurMmAt100mph(strobe.pulseWidthUs));
        cv::putText(displayLeft, benchBuf, cv::Point(20, 155), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 165, 255), 2);
      }
      if (currentMode == VIEW_GLINT_HIGHLIGHT) {
        char dotBuf[96];
        snprintf(dotBuf, sizeof(dotBuf), "Clusters: %zu | Dots rejected: %zu", dotsL.clusters.size(),
                 dotsL.rejectedDots.size());
        cv::putText(displayLeft, dotBuf, cv::Point(20, 125), cv::FONT_HERSHEY_SIMPLEX, 0.6,
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
      char lightBuf[128];
      snprintf(lightBuf, sizeof(lightBuf), "Mean R %.1f L %.1f | Max R %.0f L %.0f (2s peak R %.0f L %.0f)",
               meanRight[0], meanLeft[0], maxRight, maxLeft, peakR, peakL);
      cv::putText(displayRight, lightBuf, cv::Point(20, 65),
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
      const int lo = safeExposure.empty() ? CameraConfig::kMinExposureLog2 : safeExposure.lo;
      const int hi = safeExposure.empty() ? CameraConfig::kMaxExposureLog2 : safeExposure.hi;
      int next = std::clamp(exposureLog2 + ((key == 'E') ? 1 : -1), lo, hi);
      if (next == exposureLog2) {
        std::cout << "[IR Strobe Debugger] Exposure stays at " << CameraConfig::exposureLog2ToUs(exposureLog2)
                  << " us: the strobe-safe range at " << frameRateHz << " fps is " << safeRangeStr() << "." << std::endl;
      } else {
        exposureLog2 = next;
        int us = CameraConfig::exposureLog2ToUs(exposureLog2);
        std::cout << "[IR Strobe Debugger] Setting exposure to " << us << " us (log2 " << exposureLog2 << ")..." << std::endl;
        applyExposure(us);
      }
    } else if (key == 'g' || key == 'G') {
      gain = std::clamp(gain + ((key == 'G') ? 5 : -5), 0, CameraConfig::kMaxGain);
      std::cout << "[IR Strobe Debugger] Setting gain to " << gain << "..." << std::endl;
      if (nodeL) nodeL->setGain(gain);
      if (nodeR) nodeR->setGain(gain);
    } else if (key == 'w' || key == 'W') {
      // 10 us steps up to the shot cap, 50 us steps beyond it in bench mode.
      const int step = (key == 'W' ? strobe.pulseWidthUs >= StrobeConfig::kMaxPulseWidthUs
                                   : strobe.pulseWidthUs > StrobeConfig::kMaxPulseWidthUs) ? 50 : 10;
      StrobeConfig next = strobe;
      next.pulseWidthUs = std::clamp(strobe.pulseWidthUs + (key == 'W' ? step : -step),
                                     StrobeConfig::kMinPulseWidthUs, pulseWidthLimitUs());
      if (next.pulseWidthUs == strobe.pulseWidthUs) {
        std::cout << "[IR Strobe Debugger] Pulse width stays at " << strobe.pulseWidthUs << " us: limit is "
                  << StrobeConfig::kMinPulseWidthUs << ".." << pulseWidthLimitUs() << " us"
                  << (benchMode ? " (bench: optical budget at the controller's overdrive ratio)."
                                : " (shot cap; press 'b' for a stationary bench test past it).") << std::endl;
      } else {
        strobe = next;
        std::cout << "[IR Strobe Debugger] Pulse width " << strobe.pulseWidthUs << " us ("
                  << std::fixed << std::setprecision(1) << strobe.dutyCycle() * 100.0 << "% duty)" << std::endl;
        sendStrobePulseWidth(serial, strobe.pulseWidthUs);
        // A wider pulse lengthens the train the exposure has to hold.
        timing = makePipelineTiming(appliedExposureUs(), frameRateHz, strobe);
        safeExposure = timing.validExposureLog2Range();
      }
    } else if (key == 'b' || key == 'B') {
      benchMode = !benchMode;
      if (benchMode) {
        std::cout << "[IR Strobe Debugger] BENCH mode ON: pulse width unlocked to " << pulseWidthLimitUs()
                  << " us for a stationary ball. Flight frames would smear; this is for judging brightness only."
                  << std::endl;
      } else {
        StrobeConfig back = strobe;
        back.clampToEnvelope();
        std::cout << "[IR Strobe Debugger] BENCH mode OFF";
        if (back.pulseWidthUs != strobe.pulseWidthUs) {
          strobe = back;
          sendStrobePulseWidth(serial, strobe.pulseWidthUs);
          timing = makePipelineTiming(appliedExposureUs(), frameRateHz, strobe);
          safeExposure = timing.validExposureLog2Range();
          std::cout << ": pulse width back to " << strobe.pulseWidthUs << " us";
        }
        std::cout << "." << std::endl;
      }
    } else if (key == 'c' || key == 'C') {
      std::error_code ec;
      std::filesystem::create_directories(captureDir, ec);
      const auto stampMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch()).count();
      const std::string stem = (captureDir / ("capture_" + std::to_string(stampMs))).string();
      bool ok = true;
      if (!leftFrame.empty())  ok = cv::imwrite(stem + "_L.png", leftFrame)  && ok;
      if (!rightFrame.empty()) ok = cv::imwrite(stem + "_R.png", rightFrame) && ok;
      nlohmann::json meta = {
          {"strobeMode", strobeStatusStr}, {"mcu", mcuStatusStr}, {"bench", benchMode},
          {"pulseWidthUs", strobe.pulseWidthUs}, {"exposureUs", appliedExposureUs()}, {"gain", gain},
          {"threshold", activeThreshold}, {"fps", negotiatedFps()},
          {"maxL", maxLeft}, {"maxR", maxRight}, {"meanL", meanLeft[0]}, {"meanR", meanRight[0]},
      };
      std::ofstream(stem + ".json") << meta.dump(2) << "\n";
      std::cout << "[IR Strobe Debugger] " << (ok ? "Saved " : "FAILED saving ") << stem << "_{L,R}.png (+ .json): max L "
                << maxLeft << " R " << maxRight << std::endl;
    } else if (key == 'p' || key == 'P') {
      CameraConfig cam = config.camera;
      cam.exposureUs = appliedExposureUs();
      cam.gain = gain;
      DotClusterConfig det = dotConfig;
      det.intensityThreshold = activeThreshold;
      StrobeConfig shot = strobe;
      const bool clampedForShots = shot.clampToEnvelope();   // bench widths never go into the config
      nlohmann::json snapshot = {{"camera", cam.toJson()}, {"detector", det.toJson()}, {"strobe", shot.toJson()}};
      std::cout << "[IR Strobe Debugger] Current settings (paste into config/golfsim.json):\n"
                << snapshot.dump(2) << std::endl;
      if (clampedForShots) {
        std::cout << "[IR Strobe Debugger] Note: bench pulse width " << strobe.pulseWidthUs << " us written as "
                  << shot.pulseWidthUs << " us; a shot needs " << strobe.pulseWidthUs / shot.pulseWidthUs
                  << "x the pulse current to match this brightness." << std::endl;
      }
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
