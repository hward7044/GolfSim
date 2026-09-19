# 08 — Ownership, Lifetime and Concurrency

**Principle:** every resource has exactly one owner; copying a resource-owning object is either well-defined or impossible (Rule of Five); data crossing threads has a stated protocol.
**Effort:** Medium. **Risk:** Medium.

---

## Findings

### O1 — Pipeline components are copied into the state machine
`SessionStateMachine` takes each component *by value* ([SessionStateMachine.hpp:123-127](../../include/Orchestration/SessionStateMachine.hpp#L123-L127)) and `main.cpp` passes named lvalues ([228-243](../../src/main.cpp#L228-L243)), so every component is constructed, then copied, then the original dies at the end of `main`. Consequences:

- `TcpJsonTransmitter` owns a socket and has **no deleted copy constructor** ([TcpJsonTransmitter.hpp](../../include/Math/TcpJsonTransmitter.hpp)). Copying after `connectToServer()` would double-`close()`. It only works because it connects lazily and the original is never used. Rule-of-Five hole.
- `StereoBallTrackerTrigger` / `StereoTriangulator` / `OpenCVMomentsTracker` copies **share** their `cv::Mat` scratch buffers (shallow, ref-counted). Two instances in two threads would race on the same pixels. Latent, not live.
- `WSAStartup`/`WSACleanup` run twice on Windows (ref-counted, harmless, but a sign).

**Fix:** with [03 D2](03_Dependency_Inversion_Open_Closed.md) components become `std::unique_ptr<Interface>` injected once. Independently: `= delete` copy on every class that owns a socket, fd, thread or scratch buffer (`TcpJsonTransmitter`, the three Math classes). `SerialPort`, `FlightRecorder`, `V4L2Driver` already do this correctly.

### O2 — `shared_ptr` where there is one owner
`ThreadManager(shared_ptr<ICameraSystem>, shared_ptr<IBufferManager>, shared_ptr<ConcreteSSM>)` ([ThreadManager.hpp:45-49](../../include/Orchestration/ThreadManager.hpp#L45-L49)); `HardwareSyncedCameraSystem::addCameraNode(shared_ptr<ICameraNode>)`; the viewer's second `shared_ptr<OV9281CameraNode>` ([02 I5](02_Interface_Segregation.md)). In every case `main()` creates the object, hands a share to one consumer, and outlives it. Shared ownership models a relationship that does not exist and hides who is responsible for shutdown order (today: `ThreadManager::stop()` shuts the camera system down, but `main` still holds it).

**Fix:** an `App`/composition object owns everything by `unique_ptr` in declaration order (which is destruction order); `ThreadManager` takes references. Shutdown order becomes a property of member layout, not of `stop()` sequencing.

### O3 — Each recorded frame is deep-copied twice after capture
1. Driver → `FrameSet` (necessary).
2. `SessionStateMachine` → `recordedFramesPool` via `copyTo` ([SessionStateMachine.hpp:244-246](../../include/Orchestration/SessionStateMachine.hpp#L244-L246)) — 2 MB per frame on the consumer thread.
3. `FlightRecorder::saveSession` → `clone()` again ([FlightRecorder.cpp:100-101](../../src/Diagnostics/FlightRecorder.cpp#L100-L101)) because the pool is reused.

The pool exists to avoid allocation; the clone exists because the pool is reused; together they cost more than one `std::move` of a freshly-allocated frame would. With `maxFramesPerShot = 2`, the 40-slot pool ([line 140](../../include/Orchestration/SessionStateMachine.hpp#L140)) is 20× oversized: 80 MB resident, plus a second 100 MB pool when `--stream` is on, plus the 32 MB ring buffer.

**Fix:** the recorder owns a small pool of `RecordedFrame`s (say `maxFramesPerShot × 4`) and hands the state machine an empty one to fill (`recorder.acquire()` / `recorder.submit(std::move(frame))`). One copy, no clone, no 80 MB. Pool size derives from config ([06](06_Configuration_And_Magic_Numbers.md)).

### O4 — "Synchronized" capture is two sequential blocking calls with no Δt check
[HardwareSyncedCameraSystem.cpp:3-17](../../src/Camera/HardwareSyncedCameraSystem.cpp#L3-L17). Interface side in [02 I7](02_Interface_Segregation.md). Policy side here: after both grabs, compare the two kernel timestamps (now available); if |Δt| > half a frame period, the pair is not a stereo pair — drop it and log at debug. Optionally grab the right camera on its own thread so both `poll()`s overlap and the pair is the *same* frame more often. Measure before doing the second part.

### O5 — Diagnostics are a call-order-dependent side channel
`detectBalls(left); getTelemetry(vision); detectBalls(right); getTelemetry(vision);` ([SessionStateMachine.hpp:221-225](../../include/Orchestration/SessionStateMachine.hpp#L221-L225)) — `latestDiag` is mutable state on the component, so the *sequence* of calls is load-bearing and the JSON is copied twice per frame. Fixed by returning diagnostics from the call that produces them ([03 D3](03_Dependency_Inversion_Open_Closed.md)).

### O6 — Serial port is written from the consumer thread and the main thread
The SSM's `std::function<void(char)>` callback runs on the consumer thread ([main.cpp:255-259](../../src/main.cpp#L255-L259)); `main` writes `'0'` at shutdown after `stop()` joins — sequential today, but nothing documents that `SerialPort` is single-threaded. Becomes explicit when the `StrobeController` ([04 S1](04_Single_Responsibility.md)) owns the port and runs its keep-alive on its own timer thread → then it needs a mutex or a single-consumer queue.

### O7 — Background worker without an exception boundary
`FlightRecorder::workerLoop` ([FlightRecorder.cpp:33-56](../../src/Diagnostics/FlightRecorder.cpp#L33-L56)) calls `processSaveTask`, whose annotation code does implicit `nlohmann::json → int/float` conversions ([lines 290-330](../../src/Diagnostics/FlightRecorder.cpp#L290-L330)). A type mismatch throws `json::type_error`; nothing catches it on that thread; an exception escaping a `std::thread` calls `std::terminate` — **the launch monitor dies because a replay overlay could not be drawn.** Also listed under [09](09_Error_Handling_And_Logging.md); the ownership point is that a background thread must own its failure.

### O8 — `GlobalLogger` leaks a raw `new` singleton
Dead code ([01 A2](01_Dead_Code_And_Leftovers.md)); mentioned so nobody copies the pattern.

---

## Plan

1. O1 — `= delete` copies now (five lines); the `unique_ptr` injection comes with 03 D2.
2. O7 — `try/catch` around the task in `workerLoop`, log, continue. Ten lines. **Do now.**
3. O2 — with refactor 06's `App` extraction.
4. O3 — after [04 S2](04_Single_Responsibility.md) splits the recorder.
5. O4 — measure Δt first ([07 plan step 1](07_Data_Model_And_Time_Base.md)); implement the pair check; consider the second thread only if the numbers say so.
6. O6 — with the `StrobeController`.

## Verification
- O1: `static_assert(!std::is_copy_constructible_v<TcpJsonTransmitter>)` in a test.
- O7: test that a `RecordedFrame` with `triggerDiag = {{"teeROI", "not-an-array"}}` is saved without killing the process (raw PNGs present, overlay skipped, error logged).
- O3: resident memory at idle (`/proc/self/status VmRSS`) drops by ≥ 70 MB; `perf`/timing of `processNextFrame` shows one fewer 2 MB copy per frame.
- O4: `--stream` metadata shows |Δt(L,R)| distribution; pairs above tolerance are counted and logged.

**Depends on:** 03, 04. **Do now regardless:** O1 deletes, O7 catch.
