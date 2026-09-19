# 06 — Configuration and Magic Numbers

**Principle:** explicit over implicit. A value that governs behaviour should be named, live in one place, and be *read* by the code it governs. A config struct that nothing reads is documentation pretending to be code.
**Effort:** Medium. **Risk:** Low. Overlaps with refactor 06's `AppConfig` — this section says what `AppConfig` must contain and what must be deleted so there is one source of truth.

---

## Findings

### C1 — A 16-argument positional constructor of bare numbers
[StereoBallTrackerTrigger.hpp:112-121](../../include/Math/StereoBallTrackerTrigger.hpp#L112-L121), called from [main.cpp:228-230](../../src/main.cpp#L228-L230):

```cpp
StereoBallTrackerTrigger(StereoCalibration(), searchRoiLeft, searchRoiRight,
                         15.0, 85.0, 0.25, 120, 65.0, 256, 0.9144, 5, 4, 4.0, 0.04, 150.0, 8500.0);
```

Thirteen unlabeled scalars of mixed units (px, ratio, 8-bit level, px, px, m, frames, frames, m/s, m, px², px²). Swapping any two `double`s compiles. Three more knobs (`disparityMin/Max`, `minZDistance`, `searchStabilityTolerance`, `searchLockFrames`) are hard-coded *inside* the constructor initialiser list and cannot be set at all. `OpenCVMomentsTracker(120, 240, 80.0, 2500.0, 0.5)` and `BallPresenceTrigger` have the same shape.

**Fix:** a `struct StereoTriggerParams` with named, documented, unit-suffixed fields and defaults; C++20 designated initialisers at the call site:

```cpp
StereoTriggerParams p{ .searchRoiLeft = {350,440,600,310}, .minBallRadiusPx = 15, .maxBallRadiusPx = 85,
                       .epipolarTolerancePx = 65, .maxDistanceM = 0.9144, .impactVelocityMps = 4.0, … };
```
Same for the tracker. Constructor takes the struct by value; unset fields keep their defaults.

### C2 — `PipelineTimingConfig`: 12 fields, 4 read, 2 helpers never called
See [01 A11](01_Dead_Code_And_Leftovers.md) for the list. The four live fields are the SSM's completion rules. The rest describe the strobe/camera design but nothing consumes them: the driver hard-codes exposure ([V4L2Driver.cpp:30](../../src/HAL/V4L2Driver.cpp#L30)), the trigger hard-codes the loss timeout, the firmware hard-codes 300 Hz and 30 µs, and `isValidTiming()` — the one function that would catch an inconsistent set — is never called.

**Fix (choose one, consistently):**
- **Wire it:** `AppConfig` owns one `PipelineTimingConfig`; the camera driver receives `cameraExposureUs`, the strobe controller receives `highStrobeRateHz/standbyStrobeRateHz`, the trigger receives `ballLossTimeoutSec`, and `isValidTiming()` is asserted once at start-up. The firmware constants become a documented contract checked against the config (send `'?'`, firmware replies with its rate/pulse width).
- **Or shrink it** to the four fields that are read.
Recommended: wire it — the fields are the right ones, and [07](07_Data_Model_And_Time_Base.md) needs `strobePulseCount × pulseIntervalMs` to drive the frame rate.

### C3 — Hidden timing constant decides the impact threshold
[StereoBallTrackerTrigger.cpp:462](../../src/Math/StereoBallTrackerTrigger.cpp#L462): `double dt = 0.001; // 1 ms` converts per-frame displacement into m/s, then compares with `impactVelocityThreshold_ = 4.0 m/s`. The real frame period is ~10 ms (100 fps design) or whatever the driver negotiated. So the effective threshold is `4 mm per frame`, and the "4 m/s ≈ 9 mph" comment is off by ~10×. Works today only because 4 mm/frame happens to be a sane "it moved" threshold — but changing `impactVelocityThreshold_` in m/s does not do what its name says.

**Fix:** either take the real Δt from frame timestamps ([07](07_Data_Model_And_Time_Base.md)) and keep the threshold in m/s, or rename the parameter to `impactDisplacementM` and delete `dt`. Not both units at once.

### C4 — Scoring weights and tolerances hard-coded in algorithm bodies
`score = epiErr + 10.0 * radDiffRatio + 50.0 * circPenalty` ([StereoBallTrackerTrigger.cpp:251](../../src/Math/StereoBallTrackerTrigger.cpp#L251)); marker area `1.0 … 80.0`, Hough `(1.0, 15.0, 50.0, 35.0, 12, 55)`, padding `10/20` ([OpenCVMomentsTracker.cpp:39,118-130](../../src/Math/OpenCVMomentsTracker.cpp#L39)); epipolar `4.0`, disparity `10–300`, marker `3.0` ([StereoTriangulator.cpp:208,218,292](../../src/Math/StereoTriangulator.cpp#L208)); `confirmFrames_ >= 3`; `searchingLogCounter_ % 60`. These are tuning parameters the analysis workflow ([`.agents/skills/camera-ball-tracker-analysis`](../../.agents/skills/camera-ball-tracker-analysis/SKILL.md)) says you adjust — but half of them cannot be adjusted without editing a `.cpp`.

**Fix:** move each into the relevant `*Params` struct from C1 with a name and a comment. No behaviour change.

### C5 — Paths and ports as string literals across layers
`"build/replays"`, `"build/shot_history.json"`, `"build/session.log"`, `"127.0.0.1"`, `9002` (main) vs `3111` (`TcpJsonTransmitter` default), `"COM3"` / `"/dev/ttyACM0"`, `115200`, retry `2 × 500 ms`, replay cap `10`, ring buffer `16`, frame pool `40`, stream chunk `50`. Refactor 06's `AppConfig` lists some of these; this section adds the rest so the list is complete: **every literal above becomes an `AppConfig` field with the current value as default.** Also record the effective config into each replay's `metadata.json` ([05 D11](05_Duplication.md)) so the Python tools stop carrying their own copies.

### C6 — The `--stream` vs shot-mode flag and `RUN_DEBUG_VIEWER` are configuration disguised as control flow
Covered by [04 S1](04_Single_Responsibility.md) and [01 A12](01_Dead_Code_And_Leftovers.md); listed so `AppConfig` gets a `Mode { LaunchMonitor, StreamRecord, LiveDebug, Replay }` enum instead of three booleans.

---

## Plan

1. C1 + C4: introduce `StereoTriggerParams`, `MomentsTrackerParams`, `StereoMatchTolerances` (the last shared with [05 D1](05_Duplication.md)). Pure refactor; defaults equal today's literals; `main.cpp` uses designated initialisers.
2. C2: wire `PipelineTimingConfig` (driver exposure, strobe rates, loss timeout) and assert `isValidTiming()` at start-up. Delete `setStrobeRateHz` if still unused.
3. C3: decide units for the impact threshold (recommend real Δt via [07](07_Data_Model_And_Time_Base.md)).
4. C5/C6: fold into refactor 06's `AppConfig`; serialise the effective config into `metadata.json`.

## Verification
- `grep -rnE "\b(0\.9144|10000|5\.0|65\.0|0\.04|8500)\b" src include` → each appears once, in a defaults struct.
- Unit test: constructing a trigger with `StereoTriggerParams{}` behaves identically to today's 16-arg call (golden test on a synthetic frame sequence — exists in `tests/`).
- `metadata.json` from a stream contains a `"config"` object; `analyze_replay_stream.py --replay <dir>` reads its thresholds from it.

**Depends on:** 01. **Pairs with:** refactor 06 (`AppConfig`), [05](05_Duplication.md), [07](07_Data_Model_And_Time_Base.md).
