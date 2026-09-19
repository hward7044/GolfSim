# 09 — Error Handling and Logging

**Principle:** one policy, applied everywhere: who reports, who decides, and what the caller can rely on. Today there are three policies (bool-and-log, log-and-continue, throw) chosen per file.
**Effort:** Small–Medium. **Risk:** Low.

---

## Findings

### E1 — Return values that nobody reads
`network.transmitLaunchData(launchData)` returns `bool`; [SessionStateMachine.hpp:277](../../include/Orchestration/SessionStateMachine.hpp#L277) discards it. `saveToShotHistory` swallows an unopenable file. `FlightRecorder` logs disk errors on its worker thread and nothing upstream ever learns replays have stopped. `HardwareSyncedCameraSystem::captureSynchronizedFrames` returns `false` for "no cameras", "left failed", "right failed" and "timeout" alike, and the producer treats all four as "sleep 1 ms".

**Fix:** decide per boundary what the caller needs. Recommended:
- **Hot path** (`captureFrame`, `checkTrigger`, `detectBalls`): `bool`/`std::optional` is right; failure is expected and cheap. Add `[[nodiscard]]`.
- **Actions with consequences** (`transmitLaunchData`, `saveSession`, `writeChar`): `[[nodiscard]] bool` **and** a counter/last-error the app can surface (`network.consecutiveFailures()`); the SSM logs a warning when a solved shot was not delivered — today that is silent.
- **Initialisation** (`initialize()`, `open()`, constructors): throw `std::runtime_error` with the cause; `main` catches once. Bool-plus-log at init just moves the `if` to every caller, and `main.cpp` already has four copies of it.

### E2 — Library classes log to the global `spdlog` default logger
`spdlog::info` in `StereoBallTrackerTrigger` state changes, the `% 60` searching heartbeat, `TcpJsonTransmitter`, `FlightRecorder`, the drivers. Two costs: (a) every Math class depends on the logging library and on the process-global logger configuration (tests had to `set_level(warn)` to be readable); (b) `spdlog::info("… {:.2f}m …", …)` formats on the hot path even when the level would discard it — `spdlog` checks level before formatting, so cost is small, but the *coupling* is the issue: the trigger cannot be used in a context that wants its events routed elsewhere (e.g., into `metadata.json`, or to the overlay).

**Fix (light):** keep `spdlog`, but (1) hot-path classes log at `debug`, not `info`; (2) one named logger per subsystem (`spdlog::get("trigger")`) so the viewer/tests can silence one without silencing all; (3) state *changes* (SEARCHING → ARMED …) are events — emit them through the typed diagnostics ([03 D3](03_Dependency_Inversion_Open_Closed.md)) and let the app decide to log them. No `spdlog` include in `include/Math/*.hpp`.

### E3 — Exceptions cross thread boundaries unguarded
[08 O7](08_Ownership_Lifetime_Concurrency.md): `FlightRecorder::workerLoop` has no `try/catch`; `nlohmann::json` conversions inside can throw. Same shape in `ThreadManager`'s producer/consumer lambdas — `processNextFrame` calls OpenCV, which throws `cv::Exception` on a bad `cv::Rect` (the ARMED search window can go negative near the frame edge: `winL(x - halfWin, …)` at [StereoBallTrackerTrigger.cpp:346](../../src/Math/StereoBallTrackerTrigger.cpp#L346) is clamped inside `extractCandidates`, but `gray_(searchRect)` in the tracker's overlap path is not obviously safe). One escaped exception = `std::terminate`.

**Fix:** every thread entry point (`workerLoop`, both `ThreadManager` lambdas) wraps its per-item work in `try { … } catch (const std::exception& e) { log; continue; }`. The frame is dropped, the monitor keeps running. Add a `cv::setBreakOnError(false)` note — OpenCV throws by default, which is what we want, as long as it is caught.

### E4 — CLI parsing fails silently
`std::atoi(argv[i])` for `--frames`, `--left-cam`, `--right-cam` ([main.cpp:115-122](../../src/main.cpp#L115-L122)): `--left-cam foo` → 0, no message; `--frames -5` → silently 50. Refactor 06 extracts `parseArgs`; this section only asks that it use `std::from_chars` and reject bad input with a message and exit code 2 (the test runner already does this).

---

## Plan

1. E3 — three `try/catch` blocks. **Do now** with [08 O7](08_Ownership_Lifetime_Concurrency.md).
2. E1 — `[[nodiscard]]` on the listed methods (compiler finds the unread returns); SSM warns on failed transmit; recorder exposes a `failedTasks()` counter the app logs at shutdown.
3. E2 — demote hot-path `info` → `debug`; named loggers; remove `<spdlog/spdlog.h>` from Math headers (only `.cpp`s).
4. E4 — with refactor 06's `parseArgs`.

## Verification
- Build with `-Werror=unused-result` (GCC/Clang) after E1: clean.
- Fault-injection test: mock `INetworkTransmitter` returns `false` → SSM logs a warning and the shot is still recorded.
- Test from 08 O7 passes; a `cv::Exception` thrown from a mock `IComputerVision` inside `ThreadManager`'s consumer does not terminate the process.
- `grep -rn "spdlog" include/Math` → nothing.

**Depends on:** nothing for E1/E3; 03 D3 for the event part of E2; refactor 06 for E4.
