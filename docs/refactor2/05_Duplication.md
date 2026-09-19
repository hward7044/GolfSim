# 05 — Duplication

**Principle:** DRY — one fact, one place. Duplicated *logic* with *different constants* is the dangerous kind, and D1 below is exactly that.
**Effort:** Medium. **Risk:** Medium — D1 is probably a functional bug today.

---

## Findings

### D1 — Two stereo-matching + triangulation implementations with different tolerances ⚠️
| | `StereoBallTrackerTrigger` (production trigger) | `StereoTriangulator` (production solver) |
| :--- | :--- | :--- |
| Rectify + `cv::triangulatePoints` | [cpp:59-95](../../src/Math/StereoBallTrackerTrigger.cpp#L59-L95) own `rectifyPoint`/`triangulateCentroid` | [cpp:169-250](../../src/Math/StereoTriangulator.cpp#L169-L250) |
| Epipolar tolerance (rectified px) | `epipolarTolerancePx_` = **65** (ctor default, and `main.cpp` passes 65) | **4.0** hard-coded, [line 208](../../src/Math/StereoTriangulator.cpp#L208) |
| Disparity window (px) | **10 – 400** (`disparityMinPx_/MaxPx_`, hard-coded in ctor) | **10 – 300** hard-coded, [line 218](../../src/Math/StereoTriangulator.cpp#L218) |
| Default calibration matrices | [cpp:34-50](../../src/Math/StereoBallTrackerTrigger.cpp#L34-L50) | [cpp:138-158](../../src/Math/StereoTriangulator.cpp#L138-L158) — identical 20 lines |
| Scratchpad members | `pt2D_L_ pt2D_R_ pt4D_ undL_ undR_ pt_temp_ und_temp_` | same seven names |

The trigger's 65 px was "fine-tuned" empirically (main.cpp comment) for the uncalibrated default rectification. The triangulator will reject any left/right pair whose rectified rows differ by more than 4 px. **If real frames need ~65 px, the trigger fires but `triangulateShot` returns zero pairs, the trajectory stays below `minPointsToSolve`, and every shot is discarded with "insufficient points".** Worth checking on the first recorded stream before anything else.

**Fix:** one `StereoGeometry` class (calibration, rectification, matching with one `MatchingTolerances{epipolarPx, disparityMin, disparityMax}`, triangulation). Trigger and solver both hold a reference to it. Default calibration is constructed in exactly one factory function.

### D2 — `LaunchData` serialised to JSON three ways with three key schemes
| Where | Keys |
| :--- | :--- |
| [TcpJsonTransmitter.cpp:164-170](../../src/Math/TcpJsonTransmitter.cpp#L164-L170) | `ballSpeed, verticalLaunchAngle, spinSpeed` |
| [SessionStateMachine.hpp:103-110](../../include/Orchestration/SessionStateMachine.hpp#L103-L110) | `ballSpeed_mph, verticalLaunchAngle_deg, spinRPM` |
| [FlightRecorder.cpp:260-267](../../src/Diagnostics/FlightRecorder.cpp#L260-L267) | `ballSpeed_mph, …` (third copy) |

**Fix:** one `to_json(nlohmann::json&, const LaunchData<…>&)` next to `LaunchData.hpp` (ADL serializer). The TCP wire format, if it must differ, is a documented adapter over that.

### D3 — `FlightRecorder` clones its input twice and formats a timestamp twice
`saveSession` [97-107](../../src/Diagnostics/FlightRecorder.cpp#L97-L107) and `saveStreamSession` [150-160](../../src/Diagnostics/FlightRecorder.cpp#L150-L160) contain the same 10-line deep-copy loop; the `localtime_r/_s` + `put_time` folder-name block appears in `saveStreamSession` and `processSaveTask`. → `cloneFrames()` and `makeSessionId()`.

### D4 — `1280 × 800` in six places
`FrameSet::preallocate(1280, 800)` in [ThreadManager.cpp:37,75](../../src/Orchestration/ThreadManager.cpp#L37), [main.cpp:215](../../src/main.cpp#L215) and the viewer; `cv::Mat(800, 1280, CV_8UC1)` ×4 in [SessionStateMachine.hpp:142-143,155-156](../../include/Orchestration/SessionStateMachine.hpp#L142-L156); `REQUESTED_WIDTH/HEIGHT` in `V4L2Driver.cpp`; `1280, 800` in `MediaFoundationDriver.cpp`. The driver *negotiates* a size at runtime and warns if it differs — the warning is the only thing connecting the six copies.

**Fix:** the driver's negotiated `FrameGeometry{width,height}` flows *out* through `ICameraSystem` and everything downstream preallocates from it. One source of truth, decided by the hardware.

### D5 — Grayscale-conversion prologue ×3
Identical `if channels()==3 … else if 4 … else` blocks in `StereoBallTrackerTrigger.cpp:194-208`, `OpenCVMomentsTracker.cpp:72-78`, and `BallPresenceTrigger.cpp`. The cameras are mono; `FrameSet` is `CV_8UC1` by construction. **Fix:** assert `CV_8UC1` at the `FrameSet` boundary and delete the three blocks; or one `toGray(const cv::Mat&, cv::Mat& scratch)` in a `Math/ImageUtil.hpp`.

### D6 — ARMED and CONFIRMING build the same search windows
[StereoBallTrackerTrigger.cpp:346-352 and 438-444](../../src/Math/StereoBallTrackerTrigger.cpp#L346-L352). → `searchWindowAround(cv::Point2d)`.

### D7 — JSON candidate record built six times
[OpenCVMomentsTracker.cpp](../../src/Math/OpenCVMomentsTracker.cpp) — covered by [04 S4](04_Single_Responsibility.md).

### D8 — Firmware 3-pulse train written twice
`fireStrobeSequence` and `fire300HzBurst` ([ino:139-190](../../firmware/strobe_controller/strobe_controller.ino#L139-L190)) — see [10](10_Firmware.md).

### D9 — The same physical constants live in `PipelineTimingConfig` *and* as positional literals
`0.9144` m (config `workingDistanceMeters` **and** trigger arg `max3DDist`), `5.0` s (config `ballLossTimeoutSec` **and** trigger member), `10000` µs (config `cameraExposureUs` **and** `V4L2Driver::DEFAULT_EXPOSURE_US`), `23.3` px (config `nominalBallRadiusPx` vs trigger's 15–85 px radius window). Covered by [06](06_Configuration_And_Magic_Numbers.md).

### D10 — Two serial-port implementations in one file, two socket-type shims
`SerialPort.cpp` `#ifdef _WIN32` halves; `TcpJsonTransmitter.cpp` `INVALID_SOCKET/SOCKET_ERROR` redefinitions. Covered by [03 D5](03_Dependency_Inversion_Open_Closed.md).

### D11 — Replay/stream folder listing logic duplicated in Python and C++
`tools/analysis/analyze_replay_stream.py` re-implements thresholds (`100`, `50 px`) that differ from the C++ defaults (`120`, `65 px`). Not code duplication in one language, but the same *fact* in two places with different values. **Fix:** the analysis tool reads `metadata.json` for the parameters the C++ used (write them there — [06](06_Configuration_And_Magic_Numbers.md)).

---

## Plan

1. **D1 first, and before hardware testing**: as an interim check, set the triangulator's tolerance to the trigger's 65 px (one literal) and confirm a recorded stream solves. Then do the `StereoGeometry` extraction properly.
2. D2, D3, D5, D6 — mechanical, one PR.
3. D4 — with [08](08_Ownership_Lifetime_Concurrency.md) (geometry flows from the driver).
4. D9, D11 — with [06](06_Configuration_And_Magic_Numbers.md).

## Verification
- After D1: unit test that `StereoGeometry::match()` is called by both trigger and solver with the *same* tolerances object; `grep -rn "4.0; // 4 pixels\|disparity > 10.0" src` → nothing.
- After D2: one `to_json` symbol; `metadata.json` and TCP payload snapshot tests.
- After D4: `grep -rn "1280" src include` → only the driver's request constant.

**Depends on:** 01. **D1 also informs:** [07](07_Data_Model_And_Time_Base.md).
