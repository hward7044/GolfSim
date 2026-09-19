# GolfSim Architecture Review — Refactor Round 2

**Scope:** full history (`b11bb7e` → `3ac262c` + the uncommitted 07 work) and the current tree: `src/`, `include/`, `firmware/`, `tests/`, `docs/`.
**Lens:** SOLID, DRY, ownership/lifetime, and "does the code do what the docs say". Not a math or algorithm review.
**Format:** one file per concern so each can be reviewed, approved and implemented on its own. Nothing here is implemented.

---

## How the drift happened (from the history)

The log shows a healthy pattern of *adding* a better abstraction whenever a design changed — and an unhealthy pattern of never *removing* the one it replaced:

| Generation | Trigger design | Commit | Status today |
| :--- | :--- | :--- | :--- |
| 1 | `ROIFrameDifferencing` → `OpticalGateTrigger` (single-camera pixel diff, `'F'` armed handshake) | `7f0dbde` Jun 20 | Deleted Aug 12, but its JSON keys are still drawn by `FlightRecorder` and its serial handshake still opens a second COM port in `ThreadManager` |
| 2 | `BallPresenceTrigger` (template-match a stationary ball) | `30c1c2b` Aug 8 | Compiled, tested, included by `main.cpp` and `ThreadManager.hpp`, **never instantiated in production** |
| 3 | `StereoBallTrackerTrigger` (3-D lock + velocity) | `335f4fb` Aug 12 | Production. Contains its own copy of the triangulation math from `StereoTriangulator` |

The same shape repeats elsewhere: `Win32Serial` → `SerialPort` (alias kept), `GlobalLogger` (stub since Jun 14), `IBufferManager` (one implementation, virtual on the hot path), `PipelineTimingConfig` (12 fields, 4 read), `OV9281Registers` (no I2C path exists). Every one of these was reasonable when written; none was retired when superseded.

Five recurring patterns worth naming, because they are what "letting the AI decide the implementation" produced:

1. **Add-only abstractions.** New interface or class per feature; old one left in place "for compatibility".
2. **Stringly-typed side channels.** Component state flows to the recorder as `nlohmann::json` with ad-hoc keys; the consumer hard-codes keys from components that no longer exist.
3. **Positional-parameter explosion.** `StereoBallTrackerTrigger` takes 16 positional arguments, 13 of them bare `double`/`int`.
4. **Config that documents but does not control.** Structs with well-commented fields that nothing reads; the real value is a literal somewhere else.
5. **Docs ahead of code.** Design docs describe hardware timers, 1000 FPS, and armed handshakes that the code does not have.

---

## What is in good shape (so it is not accidentally "fixed")

- The layer split (`HAL` → `Camera` → `Math` → `Orchestration` → `Diagnostics`) is right and the directory layout matches it.
- Constructor injection through interfaces is the established pattern; refactor 07's test doubles slot in without production hooks.
- `AtomicRingBuffer` is a correct, well-reasoned SPSC design with swap-based zero-copy transfer.
- `Units.hpp` strong types and the C++20 concepts on `SessionStateMachine` show intent that the rest of the code should catch up to.
- Zero-allocation *intent* on the hot path is stated everywhere; §05/§08 are about making it true.
- Tests exist for every Math class and now run in Release, sandboxed, with a coverage gate.

---

## Sections

| # | File | Principle | Findings | Effort | Risk | Suggested order |
| :---: | :--- | :--- | :---: | :---: | :---: | :---: |
| 01 | [Dead code and leftovers](01_Dead_Code_And_Leftovers.md) | YAGNI | 14 | S | Low | **1st** — deletions only; shrinks everything after it |
| 02 | [Interface segregation](02_Interface_Segregation.md) | ISP, LSP | 7 | S–M | Low | 2nd |
| 03 | [Dependency inversion and open/closed](03_Dependency_Inversion_Open_Closed.md) | DIP, OCP | 5 | M | Med | 4th |
| 04 | [Single responsibility](04_Single_Responsibility.md) | SRP | 5 | M–L | Med | 5th |
| 05 | [Duplication](05_Duplication.md) | DRY | 11 | M | **Med — one likely functional bug** | **2nd** (item D1 first) |
| 06 | [Configuration and magic numbers](06_Configuration_And_Magic_Numbers.md) | Explicit over implicit | 6 | M | Low | 3rd (pairs with refactor 06 `AppConfig`) |
| 07 | [Data model and time base](07_Data_Model_And_Time_Base.md) | Make invariants explicit | 4 | M | **High — correctness of every 2-frame shot** | **before hardware testing** |
| 08 | [Ownership, lifetime, concurrency](08_Ownership_Lifetime_Concurrency.md) | Rule of Five, RAII | 8 | M | Med | 6th |
| 09 | [Error handling and logging](09_Error_Handling_And_Logging.md) | One policy | 4 | S–M | Low | 7th |
| 10 | [Firmware](10_Firmware.md) | Do what the safety doc says | 5 | M | **High — safety claim not implemented** | **before hardware testing** |
| 11 | [Documentation drift](11_Documentation_Drift.md) | Docs describe the code | 6 | S | Low | last |

Effort: S < ½ day, M = 1–2 days, L > 2 days. Each file ends with a verification section and its dependencies on other sections or on refactor 06/07.

**Read first if you only read two:** [07](07_Data_Model_And_Time_Base.md) and [10](10_Firmware.md). Both affect what you will see when you start hitting balls. [05 D1](05_Duplication.md) is the third.
