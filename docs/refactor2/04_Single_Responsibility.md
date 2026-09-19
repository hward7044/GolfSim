# 04 — Single Responsibility

**Principle:** SRP — a class has one reason to change. The test for it here is simple: list what a file *does* and count the unrelated verbs.
**Effort:** Medium–Large (mostly extraction, little new logic). **Risk:** Medium — these are the core classes; do it after 01–03 shrink them.

---

## Findings

### S1 — `SessionStateMachine` does six jobs
[SessionStateMachine.hpp](../../include/Orchestration/SessionStateMachine.hpp) (300 lines, header-only):

| Job | Lines | Belongs to |
| :--- | :--- | :--- |
| Shot state machine (idle → in-shot → solve) | 195-298 | **stays** |
| Pre-allocated frame pool for recording (40 × 2 × 1 MB) | 139-144, 241-252 | a `ShotFrameBuffer` |
| Stream-recording mode (a second pool + chunk flush) | 77-81, 150-161, 179-193 | a separate `IFrameConsumer` (`StreamRecorder`) — it bypasses the state machine entirely |
| Strobe command policy (`'H'`/`'L'` de-dup + callback) | 82-91, 205-213 | a `StrobeController` with its own keep-alive (fixes the watchdog bug found earlier) |
| Shot history file I/O (`shot_history.json`) | 101-119 | a `ShotHistoryWriter` |
| Diagnostics harvesting (`getTelemetry` ×3 per frame) | 93-99, 203, 222-225 | goes away with [03 D3](03_Dependency_Inversion_Open_Closed.md) |

Stream mode is the clearest case: `if (streamRecordingMode) { … return; }` at the top of `processNextFrame` means the class is really two classes selected by a flag. With [03 D1](03_Dependency_Inversion_Open_Closed.md)'s `IFrameConsumer`, `--stream` just wires a different consumer.

### S2 — `FlightRecorder` does five jobs
[FlightRecorder.cpp](../../src/Diagnostics/FlightRecorder.cpp) (471 lines): background worker/queue; deep-copying frames; folder naming (timestamp formatting twice); retention policy (`enforceLimit`, cap of 10); PNG writing; **overlay rendering** (≈ 170 lines of `cv::rectangle`/`putText` driven by JSON keys); `metadata.json` schema. Drawing is the odd one out — a replay annotator has nothing to do with async disk I/O, and it is the part coupled to component internals ([03 D3](03_Dependency_Inversion_Open_Closed.md)).

**Fix:** `ReplayAnnotator` (pure: `cv::Mat annotate(const RecordedFrame&)`), `ReplayStore` (paths, naming, retention), `FlightRecorder` keeps only the queue/worker and calls the two. Retention cap becomes a constructor parameter instead of the literal `10` at [line 72](../../src/Diagnostics/FlightRecorder.cpp#L72).

### S3 — `StereoBallTrackerTrigger::checkTrigger` is a 350-line function with three states inline
[StereoBallTrackerTrigger.cpp:188-532](../../src/Math/StereoBallTrackerTrigger.cpp#L188-L532): `if SEARCHING {…150 lines…} else if ARMED {…80…} else if CONFIRMING {…90…} else if CAPTURED {…}`. Each branch also rebuilds its `latestDiag_` JSON. The class additionally owns a private triangulator ([05 D1](05_Duplication.md)) and the emitter-standby timer.

**Fix:** one method per state (`onSearching`, `onArmed`, `onConfirming`) returning the next state; a `switch` in `checkTrigger`. Extract the standby timer into the `StrobeController` from S1 (the trigger reports "ball present / absent"; the policy of when to drop to 10 Hz is not a vision concern). Delegate triangulation to an injected `StereoTriangulator&`.

### S4 — `OpenCVMomentsTracker::detectBalls` is 40 lines of vision and 130 lines of JSON
[OpenCVMomentsTracker.cpp:62-237](../../src/Math/OpenCVMomentsTracker.cpp#L62-L237): six near-identical blocks build a candidate record (`centroid, boundingBox, area, circularity, isOverlapping, accepted, reason, markers`) inline. The algorithm is hard to see.

**Fix:** a `CandidateReport` struct (typed, not JSON) and one `report.add(contour, verdict, reason)` helper. `to_json(CandidateReport)` lives beside it for `metadata.json`. Pairs with [03 D3](03_Dependency_Inversion_Open_Closed.md).

### S5 — `main.cpp` (refactor 06 scope; recorded for completeness)
Arg parsing + composition root + production loop + 300-line strobe debugger + 130-line replay viewer. Refactor 06 covers the split; [03 D4](03_Dependency_Inversion_Open_Closed.md) says do it after D1/D2.

---

## Plan

1. S1 in this order, each a compilable step: extract `ShotHistoryWriter` (tiny, no behaviour change) → extract `StreamRecorder` as its own `IFrameConsumer` and delete the flag → extract `StrobeController` (and give it the 5 s keep-alive ping that the firmware watchdog needs) → leave the frame pool for [08](08_Ownership_Lifetime_Concurrency.md).
2. S2: extract `ReplayAnnotator` first (pure function, easy to unit-test against fixture frames — coverage Phase C1 wants exactly this), then `ReplayStore`.
3. S3: mechanical split into per-state methods; no logic change; diff should be indentation-only inside each state.
4. S4: with 03 D3.

## Verification
- `ctest` green after every step (each extraction is behaviour-preserving).
- Line counts: `SessionStateMachine` ≤ 150, `FlightRecorder.cpp` ≤ 200, no function > 80 lines in `StereoBallTrackerTrigger.cpp` (`wc -l`, or clang-tidy `readability-function-size`).
- `--stream` still produces `stream_*` folders; `--live`/production unchanged.

**Depends on:** 01, 03. **Feeds:** 08, refactor 06, coverage Phase C1.
