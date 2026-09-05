# Refactor 06: Architecture Modularity & Application Decoupling

This document outlines the architectural plan for breaking down monolithic files, centralizing configuration parameters, implementing offline replay simulation nodes, and cleanly decoupling GUI tools from the headless tracking daemon.

---

## 1. Why `main.cpp` Refactoring is Deferred to Phase 5

`src/main.cpp` currently stands at 648 lines, acting as the entry point that initializes every subsystem in the repository.

```
                    [ Monolithic main.cpp (648 lines) ]
    ┌──────────────┬───────────────┼───────────────┬────────────────┐
    ▼              ▼               ▼               ▼                ▼
Startup Lints &  Math Tests     CLI Parsing     Live Strobe      Shot Replay
Environment      Execution                      Debugger GUI     Viewer GUI
```

**Strategic Decision**:
Attempting to refactor `main.cpp` first creates an unstable moving target, as all calling signatures are being updated. We will **leave `main.cpp` untouched** through Phases 1–4, and execute this refactoring in **Phase 5** once the underlying ring buffer, geometry solvers, and hardware drivers are proven and test-covered.

---

## 2. Extraction & Modularization Architecture

```
                                [ src/main.cpp (~60 lines) ]
                                      │ (CLI Dispatch)
              ┌───────────────────────┼────────────────────────┐
              ▼                       ▼                        ▼
       [ CLI Execution ]      [ Interactive Tools ]    [ Launch Monitor Daemon ]
       --test: GolfSimTests   • CameraDebugger         • ThreadManager
       --version              • ReplayViewer           • SessionStateMachine
```

### 2.1 Module Extractions

#### 1. Centralized Configuration: `include/App/AppConfig.hpp`
Extract all magic numbers, default IP ports, serial parameters, and ROI definitions into a typed configuration structure:
```cpp
struct AppConfig {
    // Optical & Physical
    double workingDistanceMeters = 0.6096; // 2.0 ft
    double stereoBaselineMeters  = 0.100;  // 100mm
    cv::Rect searchRoiLeft       = cv::Rect(350, 440, 600, 310);
    cv::Rect searchRoiRight      = cv::Rect(350, 440, 600, 310);

    // Strobe & Timing
    double pulseIntervalMs       = 1.0;
    int    minPointsToSolve      = 3;
    int    emptyFrameTimeout     = 1;

    // Hardware & Serial
    int    leftCamIdx            = 1;
    int    rightCamIdx           = 0;
    std::string comPort          = "COM3";
    int    baudRate              = 115200;

    // Telemetry Network
    std::string simIp            = "127.0.0.1";
    int    simPort               = 9002;
};
```

#### 2. Replay Viewer Tool: `include/App/ReplayViewer.hpp` & `src/App/ReplayViewer.cpp`
Extract the 100-line `runReplayViewer()` function out of `main.cpp` into a dedicated diagnostic viewer class.

#### 3. Live Strobe Debugger: `include/App/CameraDebugger.hpp` & `src/App/CameraDebugger.cpp`
Extract the 230-line `runCameraDebugViewer()` function into an independent calibration and strobe test utility.

#### 4. Headless Simulation: `Camera/PlaybackCameraNode.cpp`
Implement the currently stubbed `PlaybackCameraNode`:
- Reads synthetic or previously recorded stereo frame pairs from disk (`build/replays/shot_.../raw/`).
- Allows the entire math and kinematics pipeline to be executed, verified, and profiled on Linux and CI environments without physical USB cameras connected.

