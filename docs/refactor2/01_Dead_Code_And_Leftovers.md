# 01 — Dead Code and Leftovers

**Principle:** YAGNI. Code that is compiled, included, tested or documented but never executed in production costs reading time, build time, test time and — worst — it *looks* like it matters.
**Effort:** Small (deletions). **Risk:** Low; every item is verified unreferenced by `grep` and by the coverage run (0 % or absent from the production binary).

---

## Findings

| ID | What | Evidence | Origin | Why it is dead |
| :--- | :--- | :--- | :--- | :--- |
| A1 | `BallPresenceTrigger` (`.hpp` 68 + `.cpp` 282 lines) | [BallPresenceTrigger.hpp](../../include/Math/BallPresenceTrigger.hpp); included by [main.cpp:35](../../src/main.cpp#L35) and [ThreadManager.hpp:5](../../include/Orchestration/ThreadManager.hpp#L5) | `30c1c2b` Aug 8 | Never instantiated. `ConcreteSSM` binds `StereoBallTrackerTrigger`. Only `tests/` uses it (60 ms of test time). |
| A2 | `GlobalLogger` + `LogLevel` | [GlobalLogger.cpp](../../src/Diagnostics/GlobalLogger.cpp) — every method body is `{}`; raw `new` singleton | `371327c` Jun 14 | Stub since creation; `spdlog` is used directly everywhere. Included by `main.cpp`, never called. |
| A3 | `Win32Serial.hpp` / `.cpp` | [Win32Serial.hpp](../../include/HAL/Win32Serial.hpp) is `using Win32Serial = SerialPort;`; the `.cpp` is a comment | `9144766` Jul 2 | Alias for a class that was renamed. Two callers (`main.cpp`, `ThreadManager.hpp`) — both can name `SerialPort`. |
| A4 | `ThreadManager::serial_` and the COM3 open in the producer thread | [ThreadManager.hpp:36-38](../../include/Orchestration/ThreadManager.hpp#L36-L38), [ThreadManager.cpp:24-33](../../src/Orchestration/ThreadManager.cpp#L24-L33) | `'F'` armed-handshake era | Opens a **second** handle on the same COM port `main.cpp` already opened (fails on Windows, logs an error every start). Nothing writes to it. Leftover from the deleted `OpticalGateTrigger` design. |
| A5 | `OV9281Registers.hpp` | [OV9281Registers.hpp](../../include/HAL/OV9281Registers.hpp) | `af8fdf4` Jun 21 | Only include is `MediaFoundationDriver.cpp`, whose register-write path is a documented no-op ("no XU on this bridge"). |
| A6 | `IUsbVideoDriver::injectImmediateRegisterWrite` | [IUsbVideoDriver.hpp:15-17](../../include/HAL/IUsbVideoDriver.hpp#L15-L17) | same | Both implementations are empty; no caller. (Also an ISP issue — see [02](02_Interface_Segregation.md).) |
| A7 | `IBufferManager<T>` | [IBufferManager.hpp](../../include/Math/IBufferManager.hpp) | `371327c` | One implementation (`AtomicRingBuffer`). Adds a virtual call to `push`/`pop` on the hot path for no polymorphism. (See [02](02_Interface_Segregation.md) for the interface angle.) |
| A8 | `PlaybackCameraNode` stub | [PlaybackCameraNode.cpp](../../src/Camera/PlaybackCameraNode.cpp) — 3 lines returning `false` | `371327c` | Placeholder for refactor 06. Included by `main.cpp`, never constructed. Either implement (06) or delete until then; a stub that compiles is worse than a TODO. |
| A9 | Unused scratchpad `cv::Mat` members | `StereoBallTrackerTrigger.hpp` [73-83](../../include/Math/StereoBallTrackerTrigger.hpp#L73-L83): `threshL_ threshR_ pt_temp_ und_temp_ undL_ undR_` (0 uses); `StereoTriangulator.hpp` [30-34](../../include/Math/StereoTriangulator.hpp#L30-L34): `undL_ undR_ pt_temp_ und_temp_` (0 uses) | copy-paste between the two classes | Declared as "zero-allocation scratchpads"; the functions that would use them allocate locals instead (`extractCandidates` creates `blurred`, `threshRoi` per call). The comment is false. |
| A10 | `lastKnownLeftRect2D_` / `lastKnownRightRect2D_` | [StereoBallTrackerTrigger.hpp:65-66](../../include/Math/StereoBallTrackerTrigger.hpp#L65-L66) | — | Written on lock and reset, never read. |
| A11 | 8 of 12 `PipelineTimingConfig` fields | [PipelineTimingConfig.hpp](../../include/Orchestration/PipelineTimingConfig.hpp): `workingDistanceMeters subPulseDurationUs nominalBallRadiusPx highStrobeRateHz standbyStrobeRateHz cameraExposureUs strobePulseCount ballLossTimeoutSec`, plus `setStrobeRateHz()` and `isValidTiming()` | refactor 02 | Nothing reads them. `main.cpp:237-241` assigns values identical to the defaults. The trigger has its own `ballLossTimeoutSec_ = 5.0`; the V4L2 driver has its own `DEFAULT_EXPOSURE_US = 10000`. (Handled properly in [06](06_Configuration_And_Magic_Numbers.md); listed here because deleting is the alternative.) |
| A12 | `RUN_DEBUG_VIEWER` | [main.cpp:57](../../src/main.cpp#L57) | pre-CLI era | `const bool … = false` compile-time toggle that duplicates `--live`. |
| A13 | `FrameSet.cpp`, `Win32Serial.cpp` | 2-line files that exist to be globbed | — | Empty translation units. |
| A14 | Stale includes in `main.cpp` and `ThreadManager.hpp` | `main.cpp` has 49 `#include`s; `ThreadManager.hpp` includes every concrete Math header | — | Fall out of A1/A2/A8 and of [03](03_Dependency_Inversion_Open_Closed.md). |

---

## Plan

1. Delete A1–A3, A5, A8, A13 files; delete the A4 member and block; remove A6 from the interface and both drivers; delete A9/A10 members; delete A12 and its `if` block.
2. `IBufferManager` (A7): delete the interface; `ThreadManager` and `main.cpp` name `AtomicRingBuffer<FrameSet, 16>` directly (they already know the type to `preallocate` it). Keep the `push`/`pop` names.
3. `PipelineTimingConfig` (A11): **do not delete yet** — resolve in [06](06_Configuration_And_Magic_Numbers.md), which wires the fields instead. If 06 is not done first, delete the eight dead fields and the two helpers so the struct is honest.
4. Remove the corresponding tests from `tests/MathTests.cpp` (`BallPresenceTrigger` case) and the name from `GOLFSIM_TEST_CASES`.
5. Re-run the include cleanup on `main.cpp` after 03 lands.

---

## Verification

- `cmake --build build` clean on Linux; **also build on Windows** (A3/A4 touch `_WIN32` paths).
- `ctest` 13/13 (one fewer case).
- `grep -rn "BallPresenceTrigger\|GlobalLogger\|Win32Serial\|OV9281Reg\|IBufferManager\|injectImmediateRegisterWrite\|RUN_DEBUG_VIEWER" src include tests` → nothing.
- Coverage report no longer lists the deleted files; production line count drops by ≈ 450.
- `./build/GolfSim` on Windows no longer logs `Trigger MCU serial port configuration failed`.

**Depends on:** nothing. **Unblocks:** everything else — do this first.
