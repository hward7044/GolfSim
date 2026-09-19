# 03 — Dependency Inversion and Open/Closed

**Principle:** DIP — high-level orchestration depends on abstractions, not on concrete Math classes. OCP — adding a trigger, a camera backend or a diagnostic should be a new file, not edits in three existing ones.
**Effort:** Medium. **Risk:** Medium (touches the composition root and the recorder).

---

## Findings

### D1 — `ThreadManager` is hard-wired to `ConcreteSSM` and, through it, to every concrete Math class
[ThreadManager.hpp:4-9, 19-25](../../include/Orchestration/ThreadManager.hpp#L4-L25): the orchestration header includes `StereoBallTrackerTrigger.hpp`, `BallPresenceTrigger.hpp`, `OpenCVMomentsTracker.hpp`, `StereoTriangulator.hpp`, `EigenBallisticsEngine.hpp`, `TcpJsonTransmitter.hpp` and defines `using ConcreteSSM = SessionStateMachine<…five concrete types…>`. `ThreadManager` only ever calls `stateMachine->processNextFrame(frameSet)`.

Consequence: swapping the trigger means editing `ThreadManager.hpp`, `main.cpp`, and every test that names `ConcreteSSM`. The "static dispatch, zero virtual overhead" comment justifies a template that saves one indirect call per frame (≤ 120/s) at the cost of the top of the dependency graph pointing at the bottom.

**Fix:** `ThreadManager` takes an `IFrameConsumer` (`virtual void processNextFrame(const FrameSet&) = 0;`). `SessionStateMachine<…>` implements it. `ThreadManager.hpp` includes nothing from `Math/`. `ConcreteSSM` moves to `main.cpp` (the composition root), which is the only place allowed to know the concrete set.

### D2 — `SessionStateMachine` template parameters buy nothing the interfaces do not already provide
Four of the five concepts are `std::derived_from<IInterface>` ([SessionStateMachine.hpp:37-47](../../include/Orchestration/SessionStateMachine.hpp#L37-L47)) — the components must inherit the virtual interface anyway, so the template only removes the `virtual` on the call while forcing every user to spell the full type. It also makes the SSM header-only (300 lines in a `.hpp`, compiled into every TU that touches it) and makes the production instantiation invisible to the tests (coverage showed `ConcreteSSM::processNextFrame` at 0 %).

**Fix:** make `SessionStateMachine` a plain class holding `std::unique_ptr<ITriggerDetector>`, `std::unique_ptr<IComputerVision>` … Move the implementation to `src/Orchestration/SessionStateMachine.cpp`. If the one virtual call per component per frame ever shows in a profile, that is the moment to reconsider — not before.

### D3 — `FlightRecorder` depends on the *string keys* of specific components, including deleted ones
[FlightRecorder.cpp:290-330](../../src/Diagnostics/FlightRecorder.cpp#L290-L330) reads `triggerDiag["teeROI"]`, `"lockedBallBox"`, `"stabilityCounter"`, `"lockFrameCount"`, `"matchScore"` (BallPresenceTrigger's keys) and `"gateROI"`, `"nonZeroCount"`, `"minBallPixels"` (the deleted OpticalGateTrigger's keys). The production `StereoBallTrackerTrigger` emits `state, leftCandidates, graceCounter, lastKnown3D, …` — **none of the drawn keys**. So the "Trigger Box (Orange)" overlay documented in `ReplayFeature.md` never renders on a real shot, and the recorder must be edited every time a component's diagnostics change (OCP violation via `nlohmann::json`).

**Fix:** invert it. Diagnostics become a small typed struct per component *or* each component exposes `void drawDiagnostics(cv::Mat& bgr) const` (an `IOverlayable`). The recorder then draws whatever it is given without knowing the keys. JSON serialisation for `metadata.json` stays, generated from the typed struct by a `to_json` in the component's TU.

### D4 — The composition root is also the application
[main.cpp](../../src/main.cpp) constructs the pipeline ([228-243](../../src/main.cpp#L228-L243)) *and* contains the 300-line strobe debugger *and* the 130-line replay viewer *and* the arg parser. Refactor 06 already plans the extraction; this section only records that D1/D2 should land **before** 06 so the extracted `LaunchMonitorApp` is built on `IFrameConsumer`, not on `ConcreteSSM`.

### D5 — Platform selection by `#ifdef` in the composition root
`PlatformCameraDriver` alias in `main.cpp` (added by refactor 08) is fine as far as it goes, but the same `#ifdef _WIN32` appears in `SerialPort.cpp` (two full implementations in one file), `TcpJsonTransmitter.cpp` (socket types), and `ThreadManager.hpp` (the dead `Win32Serial`). Platform code is scattered across layers.

**Fix:** one `src/HAL/Platform{Linux,Windows}/` directory each, selected once in CMake (`if(WIN32) list(APPEND …)`), no `#ifdef` outside `HAL/`. Small, mechanical; do after 01.

---

## Plan (order matters)

1. **D1** — introduce `IFrameConsumer`; `ThreadManager` and its tests use it. `ConcreteSSM` alias moves to `main.cpp`.
2. **D2** — de-template `SessionStateMachine`; components injected as `unique_ptr<Interface>`. Tests inject the existing mocks (they already derive from the interfaces except the trigger — fix with [02 I2](02_Interface_Segregation.md)).
3. **D3** — typed diagnostics. Do it in three steps so each compiles: (a) add `struct TriggerDiagnostics { StereoTriggerState state; std::optional<cv::Rect> searchWindowL, searchWindowR; Eigen::Vector3d lastKnown3D; … }` returned from `checkTrigger` via an out-param or a `lastDiagnostics()` accessor; (b) recorder draws from the struct; (c) delete the JSON keys the recorder no longer reads. Same for `OpenCVMomentsTracker` (`CandidateReport` — see [04 S4](04_Single_Responsibility.md)).
4. **D5** — move platform files; delete `#ifdef`s outside `HAL/`.
5. Then refactor 06.

## Verification
- `grep -rn "#include \"Math/" include/Orchestration/ThreadManager.hpp` → nothing.
- `grep -rn "_WIN32\|__linux__" src include | grep -v "^src/HAL\|^include/HAL"` → nothing after D5.
- Replay a recorded shot: the trigger overlay now draws for `StereoBallTrackerTrigger`.
- Coverage: `SessionStateMachine::processNextFrame` for the production configuration is > 0 % (it is now one class, not an uninstantiated template).

**Depends on:** 01, 02 (I2). **Blocks:** refactor 06, [04](04_Single_Responsibility.md).
