# 07 — Data Model and Time Base

**Principle:** make invariants explicit. If correctness depends on "the camera runs at exactly 100 fps and the strobe fires 3 pulses at the start of every exposure", the code must either enforce that or carry real timestamps so it does not matter.
**Effort:** Medium. **Risk:** **High** — this decides whether a 2-frame shot reports the right ball speed. Read before hardware testing.

---

## Findings

### T1 — Observations carry no time; the kinematics solver assumes uniform spacing
`BallObservation` ([IComputerVision.hpp](../../include/Math/IComputerVision.hpp)) and `Ball3D` ([ISpatialSolver.hpp](../../include/Math/ISpatialSolver.hpp)) have position only. [EigenBallisticsEngine.cpp:258](../../src/Math/EigenBallisticsEngine.cpp#L258) builds the regression with `tk = k * dt`, `dt = pulseIntervalMs` — every point is assumed exactly one strobe interval after the previous one.

How the frames are actually produced ([firmware:139-171](../../firmware/strobe_controller/strobe_controller.ino#L139-L171)): the Arduino fires **3 pulses at 0 / 3.33 / 6.67 ms after each camera STROBE edge**, then nothing until the next frame. `SessionStateMachine` concatenates the points from frame 1 and frame 2 into one `trajectoryBuffer` ([SessionStateMachine.hpp:232](../../include/Orchestration/SessionStateMachine.hpp#L232)). So the true timeline of a 2-frame shot is

```
frame 1:  0.00  3.33  6.67          frame 2:  T  T+3.33  T+6.67      (T = frame period)
assumed:  0.00  3.33  6.67          10.00  13.33  16.67
```

The assumption holds **only if T = 10.00 ms exactly** (100 fps). Refactor 02 designed around that number, but:
- nothing sets the camera to 100 fps — [V4L2Driver::selectMaxFrameRate](../../src/HAL/V4L2Driver.cpp#L363) picks the *fastest* advertised interval (OV9281 at 1280×800 is typically 120 fps → T = 8.33 ms; with a 10 ms exposure the driver may clamp exposure or drop to 60 fps — either way T ≠ 10);
- the Windows path never sets a rate at all;
- the strobe and frame clocks are locked (STROBE pin) so the *intra*-frame spacing is right, but the *inter*-frame gap is `T − 6.67 ms`, not `3.33 ms`.

At 120 fps the second frame's points are placed 1.67 ms too late in the regression → speed over-estimated by ≈ 10–15 % and the launch angle biased. At 60 fps (16.7 ms period) the error is ≈ 40 %. And with `maxFramesPerShot = 2` this is *every* iron shot.

Secondary: the number of pulses per frame is not guaranteed to be 3 (a pulse can straddle the exposure edge), so the "k-th point is at k·dt" mapping can also be off by one pulse.

**Fix:** carry time. `BallObservation` gains `uint64_t frameTimestampUs` (from `FrameSet::timestamp`, now populated by refactor 08) and `int pulseIndexInFrame` (from sorting along the principal axis, which `StereoTriangulator` already computes). `Ball3D` gains `double tSec = frameTime + pulseIndex × pulseInterval`. `IKinematicsSolver::solveKinematics(const std::vector<Ball3D>&)` — the `pulseIntervalMs` parameter disappears; the regression uses `tk = ball.tSec`. This removes the 100 fps invariant entirely.

### T2 — The frame rate is a silent free parameter
Even with T1 fixed, the *design* wants `T = strobePulseCount × pulseIntervalMs` so pulses tile the timeline. Nothing requests that rate from the driver and nothing verifies it. `PipelineTimingConfig::isValidTiming()` checks exposure ≥ train length but not frame period.

**Fix:** `ICameraSystem` exposes the negotiated frame period; `AppConfig` derives the requested fps from the timing config ([06 C2](06_Configuration_And_Magic_Numbers.md)); start-up logs *requested vs achieved* and refuses (or warns loudly) on mismatch > 5 %. The V4L2 driver's `selectMaxFrameRate` becomes `selectFrameRate(hz)`.

### T3 — The trigger's velocity estimate uses a fictional 1 ms Δt
[06 C3](06_Configuration_And_Magic_Numbers.md). With frame timestamps available (T1), `checkTrigger` receives the `FrameSet` (or its timestamp) instead of two bare `cv::Mat`s and computes Δt for real. That also fixes the interface: `checkTrigger(const FrameSet&)`.

### T4 — Strong units stop at the boundary
`Units.hpp` gives `Degrees`, `Radians`, `MilesPerHour`, `MetersPerSecond`, and `LaunchData` is templated on them — but `spinRPM` is a raw `double` in the same struct, and every internal API passes raw `double`s named `…Ms`, `…Us`, `…Px`, `…Meters`, `…Sec` (`pulseIntervalMs`, `max3DDist`, `impactVelThresh`, `ballLossTimeoutSec`, `getLastFrameTimestampUs`). The type system is used for the output and abandoned for the inputs, which is where C1's positional-argument bug class lives.

**Fix (scoped):** add `Meters`, `Seconds`, `Microseconds`, `Pixels` tags to `Units.hpp` and use them in the `*Params` structs from [06 C1](06_Configuration_And_Magic_Numbers.md) and in the timestamp fields from T1. Do not chase every internal `double` — the hot-path math stays `double`; the *boundaries* (constructors, config, timestamps) get types. Use `std::chrono::microseconds` for time rather than a home-grown tag.

---

## Plan

1. **Measure first (½ hour, before any code):** record a `--stream` on Linux and print `Δt` between consecutive `metadata.json` frame timestamps. That number is T. If it is not 10.0 ms, T1 is live.
2. T1: add timestamp + pulse index to the observation types; thread `FrameSet::timestamp` through `SessionStateMachine → vision → triangulator`; change the solver signature; update the two kinematics tests to pass explicit times (they currently pass uniformly spaced points, so results are unchanged).
3. T2: driver `selectFrameRate(hz)`; `ICameraSystem::framePeriod()`; start-up check.
4. T3: `checkTrigger(const FrameSet&)`.
5. T4: `Units.hpp` extension, applied in the params structs only.

## Verification
- Kinematics unit test with two frames at T = 8.33 ms and T = 16.7 ms: speed error < 1 % after T1 (it will be 10–40 % before).
- `--stream` metadata shows frame Δt equal to the configured `strobePulseCount × pulseIntervalMs` ± 5 % after T2, and the start-up log prints requested/achieved fps.
- Trigger test asserting the impact threshold in m/s produces the same verdict at two different frame rates.

**Depends on:** refactor 08 timestamps (done), [06 C2](06_Configuration_And_Magic_Numbers.md). **Interacts with:** [05 D1](05_Duplication.md) (same code path).
