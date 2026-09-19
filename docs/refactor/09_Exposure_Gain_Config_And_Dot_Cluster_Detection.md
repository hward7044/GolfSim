# Refactor 09: Camera Exposure/Gain Configuration & Dot-Cluster Ball Detection

**Status: IMPLEMENTED (2026-09-19) — awaiting the first real sweep (step 4) and a Release-build frame-rate measurement.**

Implemented: `CameraConfig` / `AppConfig` / `config/golfsim.json` with CLI overrides; gain, brightness, auto-off, exposure readback and media-type enumeration + frame-rate selection in both drivers; `DotClusterFinder` + `DotClusterTracker` replacing silhouette detection in the vision stage and both triggers; search ROIs removed; stereo gates from config; timing validation at startup; applied settings persisted into every replay's `metadata.json`; stream recordings annotated; live viewer shows the production detector with `e`/`E`, `g`/`G`, `+`/`-`, `p` keys; 15/15 tests. Verified on the Windows rig (both OV9281s) via a verification harness — see §7.

The launch monitor is designed to run in a **dots-only** imaging regime: camera exposure and gain are set low enough that ambient light falls away to black, and the only thing the sensor sees is the cluster of retroreflective dots on the ball lit by the IR emitter. Two things currently stop that from working:

1. **Exposure and gain are never applied in production.** `PipelineTimingConfig::cameraExposureUs` exists but is dead — nothing reads it except a test. The Windows driver has no gain control at all.
2. **The detector looks for a silhouette, not dots.** `OpenCVMomentsTracker` and `StereoBallTrackerTrigger` both threshold and then demand a large, circular contour. A correctly exposed dots-only frame has no such contour, so the better the exposure gets, the more certainly detection fails.

This document covers both fixes: **Part A** makes exposure/gain a first-class configuration that reaches the hardware; **Part B** replaces silhouette detection with dot-cluster detection.

---

## 1. Evidence

### 1.1 Live viewer screenshot (2026-09-19)

Threshold-mask view, left/right side by side, `Exp: 2000us`, `Thresh: 135`, strobe on, right-camera raw mean 64.5, 30.3 FPS. Measured from the screenshot and converted back to sensor pixels (viewer tiles are 0.5× the 1280×800 frame; display is at 125 % DPI, so 1 screenshot px = 1.588 sensor px):

| Feature | Left camera (sensor px) | Right camera (sensor px) |
| --- | --- | --- |
| **Dot cluster** centre | (667, 364) | (831, 449) |
| Cluster extent | ~14 × 20 px | ~22 × 21 px |
| Largest merged dot blob | 11 × 11 px | 21 × 16 px |
| Satellite dots | 4 blobs of 2–5 px | 2 blobs of 3–5 px |
| **Noise bar** (specular reflection) | 19 × 59 px at (684, 260), area ≈ 670 px² | 5 × 56 px at (772, 353) |

Observations that drive the design:

- **The visible cluster is ~20–22 px across — about 0.45× the nominal 46.7 px ball diameter.** This is physics, not a tuning problem: retroreflection is strongest where the ball's surface normal points back at the emitter/camera, so only the central cap of the ball lights up. The detector must expect a cluster roughly *half* a ball wide, not a full disc.
- At full sensor resolution the merged 11×11 blob will resolve into several individual dots of roughly 3–10 px² each. Expect **4–10 distinct dots** per ball per frame.
- **The noise bar is a near-miss false ball for the current trigger.** Area ≈ 670 px² passes the 150–8500 band, circularity ≈ 0.35 passes the 0.25 gate, and its equivalent radius (14.6 px) is only just under the 15 px minimum. A small exposure change would let it through. Its 3:1 aspect ratio is the reliable discriminator — the dot-cluster detector must reject elongated blobs explicitly.
- **Stereo geometry does not match the trigger's gates.** Vertical offset between the two views is a consistent ~85–93 px (both the cluster and the bar show it), which exceeds `epipolarTolerancePx_ = 65`. Horizontal disparity `xL − xR = −164 px`, but the gate requires `+10 … +400`. And the left cluster sits at y = 364, **above** the search ROI (`y = 440 … 750`). Even with a perfect detector, the current trigger would reject this ball three separate ways. See §3.7.
- A raw mean of 64.5 at threshold 135 is a thin margin. The background needs to sit well below threshold with room to spare; that is what the exposure/gain sweep is for.

### 1.2 Code audit

| Finding | Location |
| --- | --- |
| `cameraExposureUs = 10000` exists but is only referenced by a test assertion | [PipelineTimingConfig.hpp](../../include/Orchestration/PipelineTimingConfig.hpp), [MathTests.cpp:722](../../tests/MathTests.cpp#L722) |
| `setExposure()` is called only from the `--live` debug viewer on the `e` key; the production and `--stream` paths never call it | [main.cpp:523](../../src/main.cpp#L523) |
| `configureTriggerAndExposureSettings()` is a no-op that logs and returns | [MediaFoundationDriver.cpp:590](../../src/HAL/MediaFoundationDriver.cpp#L590) |
| No gain control on Windows — `IAMVideoProcAmp` is never queried | `MediaFoundationDriver.cpp` (absent) |
| Exposure µs → UVC log2 uses `floor()`, so 5000 µs becomes 3906 µs and 10000 µs becomes 7812 µs, silently | [MediaFoundationDriver.cpp:565](../../src/HAL/MediaFoundationDriver.cpp#L565) |
| Linux applies a 10 ms default exposure and disables autogain, but never sets a gain value | [V4L2Driver.cpp:431-460](../../src/HAL/V4L2Driver.cpp#L431-L460) |
| `IUsbVideoDriver` has `setHardwareExposure` only | [IUsbVideoDriver.hpp](../../include/HAL/IUsbVideoDriver.hpp) |
| Detector: threshold → contour → area 80–2500 → circularity ≥ 0.5. Replicated on a synthetic dots-only frame: 9 contours found, all rejected `Area too small`, **0 balls** | [OpenCVMomentsTracker.cpp:79-95](../../src/Math/OpenCVMomentsTracker.cpp#L79-L95), [main.cpp:224](../../src/main.cpp#L224) |
| Trigger: same silhouette assumptions (area 150–8500, radius 15–85, circularity ≥ 0.25) | [StereoBallTrackerTrigger.cpp:143-160](../../src/Math/StereoBallTrackerTrigger.cpp#L143-L160) |
| Emitter is a digital MOSFET gate — no drive-level control exists, so image brightness is set by exposure and gain only. In this project **"intensity" means the binarisation threshold** (`Thresh: 135` in the viewer): a pixel above it is 1/white, below it is 0/black | [strobe_controller.ino](../../firmware/strobe_controller/strobe_controller.ino), [main.cpp:376-380](../../src/main.cpp#L376-L380) |
| Refactor 06 planned a centralised `AppConfig` but it has not been built | [06_Architecture_Modularity.md](06_Architecture_Modularity.md) |

### 1.3 Measured camera control ranges

Arducam OV9281, 1280×800 YUY2, over UVC (probed 2026-09-19 with `tools/ir_exposure_sweep.py`). These are the same UVC controls both C++ drivers use.

| Control | Usable range | Behaviour |
| --- | --- | --- |
| Exposure | `-13 … -5` (log2 s) | Real, monotonic response: mean 22 → 79. Above `-5` the frame period clamps it and `-4 … -1` change nothing. Achievable values are therefore **122, 244, 488, 976, 1953, 3906, 7812, 15625, 31250 µs** — nothing in between. (The device reports a range of `-13 … -1`; the upper steps are clamped by the frame period.) |
| Gain | `0 … 100` | Strong: mean 24 → 111 at 977 µs exposure. Values > 100 rejected. Compensates when a longer exposure lets in more ambient. |
| Brightness | `0 … 64` | Additive black level (mean 24 → 103). Lifts exactly the floor dots-only imaging needs suppressed. **Pin to 0.** |

---

## 2. Part A — Exposure & Gain Configuration

### 2.1 Goals

- One typed `CameraConfig` that every code path (production, `--stream`, `--live`) applies to both cameras at startup.
- Loadable from a JSON file, overridable from the CLI, with compiled defaults as the fallback.
- Gain reaches the hardware on both platforms.
- The value actually applied (after UVC quantisation) is logged and readable back, so the sweep tool, the viewer, and `metadata.json` all agree on what the camera was doing.

### 2.2 `CameraConfig`

New header `include/Camera/CameraConfig.hpp`:

```cpp
struct CameraConfig {
    int  exposureUs      = 7812;   // Smallest UVC step that holds a 3-pulse 300 Hz train (see below)
    int  gain            = 0;      // 0..100, UVC gain units
    int  brightness      = 0;      // Additive black level -- keep at 0 for dots-only
    int  targetFps       = 100;    // Media type / frame interval to negotiate (see 2.5)
    bool autoExposure    = false;  // Always off in production
    bool autoGain        = false;

    // Serialisation (nlohmann::json), with every field optional so a partial
    // config file only overrides what it names.
    static CameraConfig fromJson(const nlohmann::json& j, const CameraConfig& base = {});
    nlohmann::json toJson() const;
};
```

`PipelineTimingConfig::cameraExposureUs` stays as the timing-model input (it feeds `isValidTiming()`), but `CameraConfig::exposureUs` becomes the single source of truth for the hardware. At startup, `main` copies `CameraConfig::exposureUs` into `PipelineTimingConfig::cameraExposureUs` and asserts `isValidTiming()` — the strobe train must fit inside the exposure window — and refuses to start otherwise, rather than silently losing pulses.

**Choosing the exposure.** The 2000 µs in the screenshot was an example that showed the dots, not a requirement. The strobe stays at 300 Hz, the firmware fires its pulses at 0, 3333 and 6666 µs after the camera's exposure-start edge, and the hardware only offers log2 steps, so the choice is:

| Exposure (UVC step) | Pulses captured per frame | Ambient light vs. 2000 µs |
| --- | --- | --- |
| 1953 µs (`-9`) | 1 | 1× |
| 3906 µs (`-8`) | 2 | 2× |
| **7812 µs (`-7`)** | **3** — the full train (6696 µs) | 4× |

**Default: 7812 µs with `strobePulseCount = 3`.** It is the closest step to 2000 µs that keeps the existing three-pulses-per-frame timing model (Refactor 02) intact, and it stays under the 10 ms frame period needed for 100 FPS. The trade-off: a 30 µs pulse deposits the same energy whatever the window length, so the **dots stay exactly as bright** while the ambient floor rises ~4×. That is compensated in the sweep by lowering gain and/or raising the intensity threshold — the sweep tool records all of it. If ambient at 7812 µs turns out to swamp the dots even at gain 0, drop to 3906 µs with `strobePulseCount = 2` (4 points across two frames still satisfies `minPointsToSolve = 3`).

### 2.3 Config file & CLI

- `config/golfsim.json` (checked in with defaults; user edits are the intended workflow):

  ```json
  {
    "camera":   { "exposureUs": 7812, "gain": 0, "brightness": 0, "targetFps": 100 },
    "detector": { "intensityThreshold": 135, "minDotArea": 2, "maxDotArea": 120,
                  "maxDotAspect": 2.5, "clusterRadiusPx": 28, "minDotsPerCluster": 3 },
    "stereo":   { "epipolarTolerancePx": 150, "swapCameras": false }
  }
  ```

- Load order: compiled defaults → `--config <path>` (default `config/golfsim.json`, missing file is not an error) → CLI flags `--exposure-us N`, `--gain N`.
- This is the first slice of Refactor 06's `AppConfig`. Build it as `include/App/AppConfig.hpp` holding `CameraConfig camera`, `DotClusterConfig detector` (Part B) and a small `StereoConfig` (§3.7) so 06 can grow it rather than replace it.

### 2.4 Driver interface

Extend `IUsbVideoDriver`:

```cpp
virtual void setHardwareExposure(int microseconds) = 0;
virtual void setHardwareGain(int gain) {}                 // 0..100; default no-op for mocks
virtual void setHardwareBrightness(int level) {}
virtual void setAutoExposure(bool enabled) {}
virtual int  getHardwareExposureUs() const { return -1; } // Actual applied value, -1 if unknown
virtual int  getHardwareGain() const { return -1; }
virtual void applyCameraConfig(const CameraConfig& cfg);  // Non-virtual helper calling the above in order
```

Order matters on UVC: disable auto first, then set exposure, gain, brightness. `applyCameraConfig` encodes that once.

**MediaFoundationDriver (Windows)**

- `setHardwareGain` / `setHardwareBrightness`: `QueryInterface(IID_IAMVideoProcAmp)` on `mediaSource_`, `GetRange` once at init to clamp, then `Set(VideoProcAmp_Gain, v, VideoProcAmp_Flags_Manual)`.
- `setAutoExposure(false)`: `IAMCameraControl::Set(CameraControl_Exposure, current, CameraControl_Flags_Manual)` — the `Manual` flag is what turns auto off; setting it explicitly at init means we no longer depend on the previous session's state.
- Exposure quantisation: replace `floor` with **round-to-nearest in log2 space**, clamp to the range reported by `IAMCameraControl::GetRange`, and store the resulting µs (`2^value × 1e6`) for `getHardwareExposureUs()`. Log at info level: `"Exposure requested 2000 us -> applied 1953 us (log2 -9)"`.
- `configureTriggerAndExposureSettings()` becomes `applyCameraConfig(config_)` — the `CameraConfig` is passed into the driver constructor (or a `setConfig()` before `initialize()`).
- Cache the `IAMCameraControl` / `IAMVideoProcAmp` pointers at init; these calls are off the hot path but there is no reason to `QueryInterface` on every call.

**V4L2Driver (Linux)**

- `setHardwareGain`: `V4L2_CID_GAIN` with `queryControlRange` clamping, mirroring the existing exposure code.
- `setHardwareBrightness`: `V4L2_CID_BRIGHTNESS`.
- Replace the `DEFAULT_EXPOSURE_US` constant with the `CameraConfig` passed in.
- `getHardwareExposureUs()`: read back `V4L2_CID_EXPOSURE_ABSOLUTE × 100`.

**OV9281CameraNode**: add `applyConfig(const CameraConfig&)`, `setGain(int)`, `getAppliedExposureUs()`, forwarding to the driver.

### 2.5 Frame rate

The timing model assumes 100 FPS, but the live viewer reports **30.3 FPS at a 2000 µs exposure** — so exposure is not what is limiting it. `MediaFoundationDriver::configureSourceReader` takes the **first** `L8` (or `NV12`) native media type it finds and never sets `MF_MT_FRAME_RATE`; `V4L2Driver` likewise never calls `VIDIOC_S_PARM`. The camera advertises several media types per pixel format at different frame rates, and the first one is whatever the device lists first.

- Add `CameraConfig::targetFps` (default 100).
- Windows: iterate **all** native media types, log each one's subtype, `MF_MT_FRAME_SIZE` and `MF_MT_FRAME_RATE` at info level (also from `logConnectedDevices`, so startup diagnostics show what the camera can do), then select the 1280×800 `L8`/`NV12` type with the highest frame rate ≥ `targetFps`, falling back to the highest available with a warning.
- Linux: `VIDIOC_ENUM_FRAMEINTERVALS` for the chosen format/size, then `VIDIOC_S_PARM` with `1/targetFps`.
- Expose `getNegotiatedFps()` on `IUsbVideoDriver`; log it at startup next to the applied exposure and gain, and show it in the viewer overlay alongside the measured FPS so a mismatch is visible.
- Exposure must stay below the frame period (`exposureUs < 1e6 / targetFps`); `isValidTiming()` gains this check. 7812 µs at 100 FPS passes.
- If no advertised mode reaches 100 FPS at 1280×800, that is a USB-bandwidth/firmware limit of the Arducam bridge and the timing model in Refactor 02 needs revisiting — the enumeration log is what answers this.

**Measured (2026-09-19, Windows rig).** Each OV9281 advertises 78 native types. At 1280×800: `NV12` and `MJPG` at **10, 15, 30, 60, 100 and 120 fps**. The old driver took index `[0]` = `NV12 @ 30 fps`, which is exactly the 30.3 fps the viewer showed. The new selection picks `NV12 1280x800 @ 100 fps` for the default target and both cameras negotiate it. The *delivered* rate could not be pinned down here: measured from recorded-frame timestamps in a **Debug** build with debug OpenCV, the 60 fps mode gave a clean 61 fps (camera-bound), while the 100 and 120 fps modes gave anything from 57 to 106 fps run to run — that is the consumer thread (finder ×2, vision ×2, JSON diagnostics, PNG writes) saturating, not the camera. Measure it with the live viewer's FPS counter in a Release build before trusting a number; `targetFps` stays at 100.

### 2.6 Wiring

- `main.cpp`: after both drivers initialise (production and `--stream` paths), call `node->applyConfig(appConfig.camera)` on each node. Log the applied exposure, gain and negotiated FPS for both cameras.
- `runCameraDebugViewer`: apply the config at start; replace the hard-coded exposure preset cycle with `e`/`E` = step exposure down/up one log2 stop and `g`/`G` = gain ∓5; overlay the **applied** exposure from `getAppliedExposureUs()` rather than the requested preset (the current overlay shows "2000us" while the hardware runs 1953 µs).
- `FlightRecorder::saveStreamSession` / `saveShot`: write the applied `CameraConfig` into `metadata.json` so a replay records what the camera was set to.

### 2.7 Tests

- `CameraConfig::fromJson` round-trips, partial JSON overrides only named fields, out-of-range gain/brightness clamp.
- `MockUsbVideoDriver` (in `tests/Mocks.hpp`) records every `setHardware*` call; a test asserts `applyCameraConfig` issues them in the order *auto-off → exposure → gain → brightness*.
- Log2 quantisation: table test that `2000 → 1953`, `5000 → 3906`, `10000 → 7812`, `3000 → 3906` (nearest, not floor), `50 → 122` (clamped to minimum).
- `isValidTiming()` rejects `exposureUs = 2000` with `strobePulseCount = 3`, accepts `7812 / 3` and `3906 / 2`, and rejects `15625` at 100 FPS (longer than the frame period).

### 2.8 Files

| File | Change |
| --- | --- |
| `include/Camera/CameraConfig.hpp`, `src/Camera/CameraConfig.cpp` | New |
| `include/App/AppConfig.hpp`, `src/App/AppConfig.cpp` | New — JSON load + CLI merge |
| `config/golfsim.json` | New |
| `include/HAL/IUsbVideoDriver.hpp` | Add gain/brightness/auto/readback |
| `include/HAL/MediaFoundationDriver.hpp`, `src/HAL/MediaFoundationDriver.cpp` | `IAMVideoProcAmp`, cached control interfaces, round-to-nearest, readback, media-type enumeration + frame-rate selection |
| `include/HAL/V4L2Driver.hpp`, `src/HAL/V4L2Driver.cpp` | `V4L2_CID_GAIN`, `V4L2_CID_BRIGHTNESS`, readback, `VIDIOC_S_PARM` |
| `include/Camera/OV9281CameraNode.hpp`, `src/Camera/OV9281CameraNode.cpp` | `applyConfig`, `setGain`, `getAppliedExposureUs` |
| `src/main.cpp` | Load `AppConfig`, apply to nodes in all three modes, viewer keys |
| `src/Diagnostics/FlightRecorder.cpp` | Persist applied config in `metadata.json` |
| `tests/MathTests.cpp`, `tests/Mocks.hpp` | Tests above |
| `CMakeLists.txt` | New sources |

---

## 3. Part B — Dot-Cluster Ball Detection

### 3.1 Physical model

At the 3.0 ft working distance the ball subtends ~46.7 px, but only its central retroreflective cap returns light to the camera. From the screenshot that cap is **~20–22 px across (≈ 0.45 × diameter)**, containing 4–10 dots of ~3–10 px² each, separated by a few pixels. Everything else in the frame should be black, with the exception of specular reflections off glossy surfaces (the 19 × 59 px bar), which are bright but **elongated** and **isolated**.

So a ball is: *several small compact blobs, all within about half a ball diameter of each other.* A ball is **not** a large circular blob.

### 3.2 Algorithm

Shared utility `include/Math/DotClusterFinder.hpp` / `src/Math/DotClusterFinder.cpp`, used by both the tracker and the trigger:

```cpp
struct DotClusterConfig {
    int    intensityThreshold  = 135;   // The "intensity" setting: pixel > this is 1/white, else 0/black. Tuned with the sweep tool
    double minDotArea          = 2.0;   // px^2 -- below this is sensor noise
    double maxDotArea          = 120.0; // px^2 -- above this is a merged/bloomed region or a reflection
    double maxDotAspect        = 2.5;   // Bounding-box aspect ratio; rejects the reflection bar
    double clusterRadiusPx     = 28.0;  // Dots within this distance belong together (~0.6 x ball diameter)
    int    minDotsPerCluster   = 3;     // Fewer than this is a stray reflection, not a ball
    int    maxDotsPerCluster   = 40;    // More than this is a lit-up background
    double maxClusterSpreadPx  = 47.0;  // Max extent of one cluster (one ball diameter)
    double minClusterSpreadPx  = 6.0;   // Min extent -- a single bloomed dot is not a ball
};

struct Dot   { cv::Point2d centroid; double area; double peak; };
struct DotCluster {
    cv::Point2d      centroid;   // Intensity-weighted mean of dot centroids
    double           spreadPx;   // Max dot-to-centroid distance x 2
    std::vector<Dot> dots;
    cv::Rect         boundingBox; // Nominal ball box: centroid +/- nominalBallRadiusPx
};

class DotClusterFinder {
public:
    explicit DotClusterFinder(DotClusterConfig cfg, double nominalBallRadiusPx);
    std::vector<DotCluster> find(const cv::Mat& gray);   // Whole frame -- no search ROI (see 3.7)
    const nlohmann::json& lastDiagnostics() const;   // Every dot and every cluster, accepted or rejected, with reasons
private:
    cv::Mat mask_;   // Zero-allocation scratch
};
```

`find()`:

1. **Threshold** the whole frame at `intensityThreshold` → binary mask. There is no search ROI: with continuous 300 Hz strobing the ball can be found anywhere in frame before the shot, when nothing is time-critical, and a full-frame threshold + contour pass on a mostly-black 1280×800 image is well under a millisecond.
2. **Extract dots**: `findContours(RETR_EXTERNAL)`. For each contour take `area = max(contourArea, contour.size())` (a 1–2 px glint has zero polygon area but is real), bounding box, aspect = `max(w,h)/min(w,h)`, sub-pixel centroid from moments, peak from `minMaxLoc` on the box. Reject with a reason if `area < minDotArea`, `area > maxDotArea`, or `aspect > maxDotAspect`.
3. **Cluster**: greedy single-linkage on centroids — sort dots, seed a cluster with the first unassigned dot, absorb any unassigned dot within `clusterRadiusPx` of the cluster's *current* centroid, repeat until stable, then next seed. With ≤ 50 dots per frame this is O(n²) on tiny n and allocation-free with a fixed-capacity buffer.
4. **Validate cluster**: reject if `dots.size() < minDotsPerCluster`, `> maxDotsPerCluster`, `spread > maxClusterSpreadPx`, or `spread < minClusterSpreadPx`.
5. **Centroid**: intensity-weighted mean (`Σ peak·xy / Σ peak`). `boundingBox` is the nominal ball circle around it, clipped to the frame — downstream code that sizes ROIs by the box keeps working.
6. Emit diagnostics for every dot and cluster, accepted or not, in the same `candidates[]` shape the `FlightRecorder` already draws.

### 3.3 Why this rejects the observed noise

| Noise | Rejected by |
| --- | --- |
| 19 × 59 px reflection bar (area ≈ 670, aspect 3.1) | `maxDotArea` **and** `maxDotAspect` — two independent gates |
| Isolated 2–5 px sensor sparkles | `minDotsPerCluster` (a lone dot is never a ball) |
| Lit-up background at too-high gain | `maxDotsPerCluster` / `maxClusterSpreadPx`, and it shows up as `bright_pct` in the sweep analyser before it ever reaches the detector |

### 3.4 Centroid bias

The dot-cluster centroid is the centre of the *lit cap*, not the geometric ball centre; the cap is offset toward the emitter. For a fixed emitter/camera geometry this offset is consistent frame to frame, so:

- **Velocity and launch angle** come from differences between frames and are unaffected.
- **Absolute 3D position** carries a small systematic bias (sub-radius). Acceptable for triggering; if it matters for carry it can be calibrated out as a constant per-camera offset later.
- **Spin** already uses the dots themselves (`MarkerObservation`), which this design supplies directly — the dots *are* the markers, so `extractMarkersInROI` and its separate `markerThreshold` go away.

### 3.5 Integration points

**`DotClusterTracker : IComputerVision, IDiagnosticProvider`** (`include/Math/DotClusterTracker.hpp`) — thin adapter: `detectBalls(frame)` → `finder_.find(gray)` → map each `DotCluster` to a `BallObservation { centroid, boundingBox, markers = dots }`. Replaces `OpenCVMomentsTracker` in `main.cpp`. Keep `OpenCVMomentsTracker` in the tree for now (Refactor 06 will decide what to prune) but stop constructing it.

**`StereoBallTrackerTrigger::extractCandidates`** — drop the `searchROI` parameter, replace the threshold/contour/circularity body with `finder_.find(gray)` and map each cluster to a `BlobCandidate { centroid, rectCentroid (rectified), boundingRect, area = Σ dot areas, radius = spread/2, circularity = 1.0 }`. The pair-scoring that follows (epipolar error, radius symmetry, 3D distance) is unchanged. Radius symmetry between views still means something: both cameras see a similar-sized cap.

**`BallPresenceTrigger`** — same substitution in `checkOpticalGate`, and its `teeROI` goes too. It is the single-camera fallback and should not be left on the old model.

**`FlightRecorder` overlay** — draw each dot as a small filled circle and each accepted cluster as a circle of `nominalBallRadiusPx` with a crosshair; rejected clusters in red with the reason. The JSON shape stays compatible with `tools/analysis/analyze_replay_stream.py`.

### 3.6 Parameter sources

`DotClusterConfig` lives in `AppConfig.detector` (§2.3). `intensityThreshold` is the one value that must be tuned together with exposure/gain — the intended workflow is:

```
python tools/ir_exposure_sweep.py --strobe continuous
python tools/analysis/analyze_ir_dots.py --sweep build/replays/sweep_<ts>
```

read `exposure`, `gain`, and `threshold` from the top row of `report.txt`, and write them into `config/golfsim.json`. The analyser's scoring already implements the §3.2 rules in Python, so the two should be kept in step: when the C++ config gains a field, the analyser gets the same flag.

### 3.7 Stereo gates: remove the ROI, widen the rest

The search ROI and the tight epipolar/disparity gates date from the reactive-trigger design, where the pipeline had to cut the pixels it analysed to keep up. Under continuous 300 Hz strobing (Refactor 03) that constraint is gone: before the shot the ball is stationary and there is no time pressure, and once it is struck the strobe is already running. So these gates lose their purpose and, as §1.1 shows, currently reject a correctly detected ball three separate ways.

| Gate | Current | Measured | Decision |
| --- | --- | --- | --- |
| Search ROI (both cams) | `y = 440 … 750` | Left cluster at y = 364, outside it | **Remove.** `DotClusterFinder::find` takes the whole frame; `searchRoiLeft_` / `searchRoiRight_` / `teeROI` and their constructor parameters are deleted, not defaulted to full-frame. |
| `epipolarTolerancePx_` | 65 | ~85 px vertical offset, consistent across features | **Raise to 150** and move it to `AppConfig.stereo.epipolarTolerancePx`. It only has to separate the one ball from stray reflections, which the finder already filters. Proper `StereoCalibration` can tighten it later; not needed now. |
| Disparity `xL − xR` | `+10 … +400` | **−164** | Widen the band to `+5 … +600` and add `AppConfig.stereo.swapCameras` (what `--swap-cameras` does today, so it is not forgotten between runs). The sign says which physical camera is registered as which; a sweep with the ball on the tee settles it — the analyser prints `xL − xR` per setting. |

Add to `analyze_ir_dots.py`: when both sides of a setting have a cluster, print the disparity and vertical offset so the two remaining checks come for free with every recording.

### 3.8 Tests

Synthetic frames are cheap to generate and deterministic; put them in `tests/MathTests.cpp` under a `DotCluster` group:

- **Ideal ball**: black field, 9 dots of 2 px radius within a 36 px circle → exactly one cluster, centroid within 0.5 px of truth, 9 markers.
- **Reflection bar**: add a 19 × 59 px rectangle → still exactly one cluster; diagnostics list the bar with reason `aspect`.
- **Lone sparkle**: single 4 px blob far from the ball → one cluster; sparkle rejected `minDotsPerCluster`.
- **Two balls** 200 px apart → two clusters, neither absorbs the other.
- **Merged dots**: two dots 1 px apart forming one 18 px² blob → still counts as one dot (not rejected), cluster still found.
- **Empty frame** → no clusters, no allocations (assert scratch buffers reused).
- **Trigger integration**: `StereoBallTrackerTrigger` with a mock calibration and two synthetic views 60 px apart horizontally → reaches `ARMED`.
- **Replay regression**: once a real sweep is recorded, commit one left/right pair under `tests/data/` and assert the cluster is found at the known position. This is the test that catches threshold drift.

### 3.9 Files

| File | Change |
| --- | --- |
| `include/Math/DotClusterFinder.hpp`, `src/Math/DotClusterFinder.cpp` | New — core algorithm + diagnostics |
| `include/Math/DotClusterTracker.hpp`, `src/Math/DotClusterTracker.cpp` | New — `IComputerVision` adapter |
| `include/Math/StereoBallTrackerTrigger.hpp`, `src/Math/StereoBallTrackerTrigger.cpp` | `extractCandidates` uses the finder; ROI members/params removed; tolerance from config |
| `include/Math/BallPresenceTrigger.hpp`, `src/Math/BallPresenceTrigger.cpp` | `checkOpticalGate` uses the finder; `teeROI` removed |
| `src/Diagnostics/FlightRecorder.cpp` | Dot + cluster overlay |
| `src/main.cpp` | Construct `DotClusterTracker`; epipolar tolerance / swap from config; ROI arguments removed |
| `tools/analysis/analyze_ir_dots.py` | Print stereo disparity / vertical offset per setting |
| `tests/MathTests.cpp` | §3.8 |
| `CMakeLists.txt` | New sources |

---

## 4. Implementation order

Each step leaves the build green and is independently verifiable.

1. **A.1 — Driver readback + round-to-nearest exposure + frame-rate negotiation** (`MediaFoundationDriver`, `V4L2Driver`). Verify with the live viewer: overlay shows 7812 µs for a 7812 µs request (and 1953 µs for 2000 µs), and the FPS counter reads ~100.
2. **A.2 — Gain + brightness + auto-off on both drivers**, `IUsbVideoDriver` extension, mock driver. Verify: `--live` with `g`/`G` visibly changes image brightness on Windows.
3. **A.3 — `CameraConfig` / `AppConfig` / `config/golfsim.json` / CLI flags**, applied in all three modes, persisted into `metadata.json`. Verify: `--stream` recordings analysed by `analyze_ir_dots.py --replay` match the sweep tool's numbers for the same setting.
4. **Record a real sweep** with the ball on the tee at 7812 µs; pick gain/intensity threshold; commit them to `config/golfsim.json` and a reference frame pair to `tests/data/`. Read the epipolar offset and disparity sign off `report.json` (§3.7). **← next: needs the rig.**
5. **B.1 — `DotClusterFinder`** + synthetic tests. Pure function, no hardware.
6. **B.2 — `DotClusterTracker`** wired into `main.cpp`; `FlightRecorder` overlay. Verify on the step-4 recording via `--replay`.
7. **B.3 — Triggers use the finder**; ROIs removed, tolerance / swap from config. Verify: `--stream` with a ball on the tee reaches `ARMED` in `session.log`.
8. **Update** `docs/ReplayFeature.md`, `docs/Project Details.md`, and this index.

Steps 1–3 and 5 have no dependency on each other and can proceed in parallel.

Steps 1, 2, 3, 5, 6 and 7 are done; step 4 needs the ball and emitter; step 8 is this document plus `docs/ReplayFeature.md` and `tools/analysis/README.md`.

---

## 5. Acceptance criteria

- With `config/golfsim.json` present, `GolfSim.exe` logs the **applied** exposure, gain and negotiated frame rate for both cameras at startup, and `metadata.json` in every replay records them.
- Both cameras negotiate 100 FPS at 1280×800 (or the startup log states the highest rate the camera advertises, so the limit is known rather than guessed).
- On Windows, changing `gain` in the config visibly changes frame intensity (verified against the sweep tool's numbers for the same setting).
- `analyze_ir_dots.py --replay` on a `--stream` recording made at the configured setting scores ≥ 80 for the ball frames.
- `DotClusterTracker` finds exactly one ball on the step-4 reference pair, at the cluster centroid measured by the analyser, with ≥ 4 markers.
- The reflection bar in that reference pair appears in diagnostics as a rejected dot, never as a candidate ball.
- `StereoBallTrackerTrigger` reaches `ARMED` with a stationary ball on the tee **anywhere in frame** within `searchLockFrameCount` frames.
- All new tests pass under CTest; the coverage gate does not regress.

---

## 6. Open questions

- **Ambient at 7812 µs.** Going from 2000 µs to 7812 µs lets in ~4× the ambient while the dots stay the same brightness. Whether gain 0 plus a higher intensity threshold is enough, or whether an 850 nm bandpass filter on the lenses is needed to kill ambient, is answered by the first sweep at 7812 µs (§4 step 4). Fall-back is 3906 µs / 2 pulses (§2.2).
- **Disparity sign.** `xL − xR = −164` in the screenshot. If the sweep confirms it, `swapCameras = true` is the fix; if it flips depending on where the ball sits, the rig is toed-in and needs a real `StereoCalibration`.
- **Does the camera advertise 100 FPS at 1280×800?** Yes — `NV12` at 100 and 120 fps (§2.5, measured). What remains open is what the *pair* sustains end to end; the Debug-build numbers were consumer-bound. Measure in Release with the viewer's FPS counter. If it falls short, the first suspects are the sequential blocking `ReadSample` in `captureSynchronizedFrames` and the per-frame JSON diagnostics in stream mode.
- **Windows build.** The tree has required OpenCV 5 since the Sep 5 refactor, but this machine only has vcpkg OpenCV 4.7, so `build/GolfSim.exe` predates every change since. `CMakeLists.txt` now defines `NOMINMAX` on Windows (it did not compile at all without it). A Windows OpenCV 5 install, or a documented Linux-only build, is needed before the exe in `build/` means anything.

---

## 7. Verification record (2026-09-19)

No OpenCV 5 exists on the Windows machine, so verification used a scratch CMake harness that builds the real `src/`, `include/` and `tests/` against the vcpkg OpenCV 4.7 already present, with a two-header shim mapping `<opencv2/geometry.hpp>` and `<opencv2/stereo.hpp>` to `<opencv2/calib3d.hpp>` (every OpenCV call the code makes exists in both). Nothing from the harness is committed.

- `GolfSimTests`: 15/15 (11 existing + `CameraConfig`, `AppConfig`, `DotClusterFinder`, `DotClusterTracker`). `BallPresenceTrigger` and `StereoBallTrackerTrigger` now use synthetic dot balls; the stereo test's "occlusion" step previously painted over the ball and was really a dual-loss frame — it now genuinely exercises single-camera immunity, so the grace window needs one more frame in the test.
- `GolfSim.exe --stream` on the rig: both cameras enumerate 78 media types, negotiate `NV12 1280x800 @ 100 fps`, report exposure range `-13..-1`, gain `0..100`, brightness `-64..64`; exposure `7812 -> 7812 us (log2 -7)`, gain `0 -> 0` applied and read back; timing validation passes; frames flow; the finder runs on real frames (0 clusters with the emitter off, a few rejected dots); stream sessions save with annotated frames and the `session` block in `metadata.json`.
- One run of the 100 fps mode delivered no frames at all, immediately after Python had held the cameras; every later run delivered. Treat as a startup race until seen again.
