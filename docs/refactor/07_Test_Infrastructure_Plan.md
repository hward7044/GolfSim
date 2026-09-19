# Refactor 07 — Implementation Plan: Test Infrastructure & Coverage

> **Status: Steps 1–7 and Phase C0 IMPLEMENTED (2026-09-19). Phases C1–C6 are open.**
> Companion to [07_Test_Infrastructure.md](07_Test_Infrastructure.md), which is the *specification* (problem statement, macro design, CTest target). This file is the *execution plan*: where the code started, the exact steps, deviations from the spec and why, and how it was proven. Revision 2 added the separation guarantees (§3) and the coverage plan (§8); revision 3 records the implementation outcome (§9).

---

## Review Resolutions (rev 2)

| Reviewer comment | Resolved in |
| :--- | :--- |
| "Make sure all test code is separate from the execution code." | §3 — five rules, the three current violations (`main.cpp` verification block, `test_xu.cpp` at repo root, mocks inside the test file), and a CTest guard that fails the build if test symbols ever leak back into `src/`/`include/`. Steps 3b, 6, 6b, 7. |
| "Create a plan to get to 100% line, statement, and logic coverage on this repo." | §8 — tool choice with the three metrics defined precisely, a **measured baseline** (61 % lines / 44 % branches / 33 % MC/DC on production code today), scope and exclusion policy, six phases with per-file gap lists and the seams each needs, and a ratcheting gate. |

---

## 0. Starting Point (measured 2026-09-19 at commit `3ac262c`, before this refactor — paths and line numbers are a snapshot)

| Fact | Where |
| :--- | :--- |
| `src/MathTests.cpp` is 944 lines: 11 test functions, **153 `assert()` calls**, 5 mock structs, one `runMathTests()` entry point | [MathTests.cpp:924-943](../../src/MathTests.cpp#L924-L943) |
| It is compiled **into the production binary** by `file(GLOB_RECURSE SOURCES "src/*.cpp")` and called on every launch | [CMakeLists.txt:26](../../CMakeLists.txt#L26), [main.cpp:177-178](../../src/main.cpp#L177-L178) |
| `main.cpp` also carries a 47-line "Build Environment Verification" block (JSON / spdlog / Eigen / OpenCV smoke tests) that runs on every launch | [main.cpp:134-180](../../src/main.cpp#L134-L180) |
| Every launch has side effects in the real data directories: **3 synthetic `shot_*` folders** land in `build/replays/` (the SSM tests use `SessionStateMachine`'s default `FlightRecorder("build/replays")`) and **3 rows** are appended to `build/shot_history.json` via `saveToShotHistory()` | [SessionStateMachine.hpp:64](../../include/Orchestration/SessionStateMachine.hpp#L64), [SessionStateMachine.hpp:101-123](../../include/Orchestration/SessionStateMachine.hpp#L101-L123) |
| Wall-clock sleeps total ≈ **1.5 s**: 60 + 60 ms (standby timeouts), 12 × 5 ms (unique folder timestamps), 800 ms + 500 ms (waiting for the async recorder) | [MathTests.cpp:93,399,543,681](../../src/MathTests.cpp#L93) |
| Linux build is `Debug`, so asserts are live. The Windows preset and any `Release` configure add `-DNDEBUG` → **every assert becomes a no-op and the suite passes vacuously** | `build/CMakeCache.txt` |
| No `tests/` directory, no `enable_testing()`. `build/Testing/` exists only from an empty CTest run on Sep 7 | — |
| Production code has no `assert()` of its own (only two `static_assert`s in `AtomicRingBuffer`) — removing `<cassert>` from the tests is safe | — |
| A stray Windows-only probe, `test_xu.cpp`, sits at the repo root (not in the build) | `./test_xu.cpp` |
| `CMakePresets.json` is gitignored (Windows-local); Windows uses a multi-config MSVC generator | `.gitignore` |
| Seven production classes have **zero** test references: `TcpJsonTransmitter`, `GlobalLogger`, `ThreadManager`, `HardwareSyncedCameraSystem`, `OV9281CameraNode`, `PlaybackCameraNode`, `V4L2Driver` | grep |

---

## 1. Goals and Non-Goals

**Goals**
1. `./build/GolfSim` starts with zero test execution and zero test side effects.
2. Assertions fail loudly in `Release` builds.
3. `ctest` runs the suite; each case is individually addressable (`ctest -R Kinematics`).
4. No test writes outside a sandbox directory; `build/replays/` and `build/shot_history.json` are never touched by tests.
5. Full suite runs in **< 300 ms** (from ≈ 1.5 s).
6. **No new third-party dependencies** for the test runner itself (coverage reporters are dev-tools, see §8.1).
7. Test code and execution code are physically and build-level separate, and stay that way (§3).
8. A measurable, ratcheting path to 100 % line / statement / logic coverage (§8).

**Non-goals (deliberately deferred)**
- Adopting Catch2 / GoogleTest. Revisit if the suite passes ~30 cases; the registry below is a two-line swap to either.
- CI pipeline — OpenCV 5 is not packaged for GitHub-hosted runners.
- Injectable clock for the two 60 ms standby-timeout tests (bounded and deterministic enough; keep).
- `PlaybackCameraNode` replay fixtures — that is refactor 06 (but §8 Phase C5 depends on it).

---

## 2. Decisions Where This Plan Deviates From the Spec

| Spec ([07](07_Test_Infrastructure.md)) says | This plan does | Why |
| :--- | :--- | :--- |
| List the production `.cpp` files explicitly inside `add_executable(GolfSimTests …)` | Build a **`golfsim_core` STATIC library** from `src/*.cpp` minus `main.cpp`; `GolfSim` and `GolfSimTests` both link it | No double compilation, no source-list drift as files are added, Windows MF/Winsock libs are declared once (`PUBLIC`) |
| `TEST_ASSERT(condition, msg)` — two arguments | `TEST_ASSERT(cond)` **plus** `TEST_ASSERT_MSG(cond, msg)` **plus** `TEST_NEAR(a, b, tol)` | 153 existing asserts carry no message → the conversion is a mechanical `sed`. ~30 of them are `std::abs(a - b) < tol`; `TEST_NEAR` prints the actual values on failure instead of just the expression text |
| Keep `src/MathTests.cpp`, reference it from the test target | **`git mv src/MathTests.cpp tests/MathTests.cpp`** | The production `GLOB` must stop seeing it; `git mv` preserves rename history for the conversion diff |
| One `add_test(NAME AllMathTests …)` | A tiny **test registry** + one `add_test` per case + `AllMathTests` | Per-case pass/fail and timing in `ctest` output; `--filter` for local iteration |
| `throw std::runtime_error` on failure | Same, but the runner **catches per case** and continues | A failure in `testUnits` should not hide a failure in `testKinematicsEngine` |

---

## 3. Separation of Test Code From Execution Code

### 3.1 Rules

1. **`tests/` is the only home for test code.** Test cases, mocks, fakes, fixtures, sandbox helpers, the runner. Nothing under `src/` or `include/` may reference `TEST_ASSERT`, `GOLFSIM_TEST`, `TestRegistrar`, or any `Mock*`/`Fake*` type.
2. **Build-level isolation.** `golfsim_core` and `GolfSim` never compile or link anything from `tests/`. `GolfSimTests` is the only consumer. With `-DGOLFSIM_BUILD_TESTS=OFF` the production binary contains no test symbols at all.
3. **Production code may expose test *seams* only in these forms** — all of which are also good production design, so nothing exists "for tests only":
   - Constructor injection of an interface (already the codebase pattern: `IUsbVideoDriver`, `ICameraNode`, `ICameraSystem`, `IBufferManager`, the five SSM template parameters).
   - Trailing defaulted constructor parameters for paths/ports (Step 4).
   - A small `SysCalls` struct of function pointers for OS-bound drivers (§8 Phase C3).
   - **Not allowed:** `#ifdef GOLFSIM_TESTING`, `friend class *Test*`, public methods that exist only so a test can poke state. (Existing setters like `setLossTimeoutSec` are configuration, not test hooks — they stay.)
4. **No verification code on the production start-up path.** Smoke checks of third-party libraries, math self-tests, and environment banners are tests; they live in `tests/` or behind an explicit CLI flag, never in the default `main()` flow.
5. **Stray experiments live in `scratch/`**, which is not part of any target.

### 3.2 Current violations this plan removes

| Violation | Fix | Step |
| :--- | :--- | :---: |
| `runMathTests()` called from `main()` | Delete the call and forward declaration | 6 |
| 47-line environment-verification block in `main()` ([main.cpp:134-180](../../src/main.cpp#L134-L180)) — literally `// Test nlohmann/json`, `// Test Eigen`, … | Delete. Keep only the library-version lines, behind a new `--version` flag (≈ 8 lines) | 6 |
| Five mock structs embedded in the test file | Move to `tests/Mocks.hpp` so the coverage phases reuse them | 3b |
| `test_xu.cpp` at repo root | `git mv test_xu.cpp scratch/test_xu.cpp` (it is a one-off Windows XU probe, not built) | 6b |

### 3.3 Enforcement (so it stays true)

Two cheap CTest cases in `tests/SeparationGuard.cmake`, registered by Step 1's CMake:

- **`NoTestCodeInProduction`** — `grep -rEn '\b(TEST_ASSERT|GOLFSIM_TEST|TestRegistrar|TestSandbox|Mock[A-Z][A-Za-z]*|runMathTests)\b' src include` must return nothing.
- **`NoTestSymbolsInBinary`** — on Linux, `nm -C $<TARGET_FILE:GolfSim> | grep -c 'TestRegistrar\|test_[A-Z]'` must be 0. (Skipped on Windows; the grep guard covers it.)

Both run in < 50 ms and fail loudly with the offending line.

---

## 4. Target Layout

```
tests/
├── TestAssert.hpp        TEST_ASSERT / TEST_ASSERT_MSG / TEST_NEAR (release-safe, throw on failure)
├── TestRegistry.hpp      GOLFSIM_TEST(name) — self-registering test cases
├── TestSandbox.hpp       TestSandbox::path("replays_test") → <CMAKE_BINARY_DIR>/test_sandbox/replays_test
├── Mocks.hpp             MockStrobeTrigger/Vision/Spatial/Kinematics/Net (moved from MathTests.cpp)
├── TestMain.cpp          runner: --list | --filter <substr> | <name>... | --verbose
├── MathTests.cpp         moved from src/; each testX() wrapped in GOLFSIM_TEST; runMathTests() deleted
└── SeparationGuard.cmake the two guard tests from §3.3
```

Splitting `MathTests.cpp` into per-subsystem files (`TriggerTests.cpp`, `RecorderTests.cpp`, …) happens naturally in §8 Phase C1 when new cases are added per module — not in this pass, so rename detection survives the `assert` → `TEST_ASSERT` diff.

---

## 5. Steps

### Step 1 — CMake: core library + test target

Replace the single-target `CMakeLists.txt` body with:

```cmake
# --- Core library: everything except the entry point ---
file(GLOB_RECURSE CORE_SOURCES "src/*.cpp")
list(FILTER CORE_SOURCES EXCLUDE REGEX "src/main\\.cpp$")
add_library(golfsim_core STATIC ${CORE_SOURCES})
target_include_directories(golfsim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(golfsim_core PUBLIC
    ${GOLFSIM_OPENCV_LIBS} Eigen3::Eigen spdlog::spdlog nlohmann_json::nlohmann_json)
if(WIN32)
    target_link_libraries(golfsim_core PUBLIC
        mfplat mfreadwrite mfuuid mf ksuser ole32 strmiids ws2_32)
endif()

# --- Production binary ---
add_executable(GolfSim src/main.cpp)
target_link_libraries(GolfSim PRIVATE golfsim_core)

# --- Tests ---
option(GOLFSIM_BUILD_TESTS "Build the GolfSimTests runner and register with CTest" ON)
if(GOLFSIM_BUILD_TESTS)
    enable_testing()
    file(GLOB TEST_SOURCES "tests/*.cpp")
    add_executable(GolfSimTests ${TEST_SOURCES})
    target_link_libraries(GolfSimTests PRIVATE golfsim_core)
    # Absolute sandbox root so the runner behaves the same from any cwd (ctest, IDE, shell)
    target_compile_definitions(GolfSimTests PRIVATE
        GOLFSIM_TEST_SANDBOX_DIR="${CMAKE_BINARY_DIR}/test_sandbox")

    set(GOLFSIM_TEST_CASES
        Units BallPresenceTrigger StereoBallTrackerTrigger OpenCVMomentsTracker
        StereoTriangulatorAndRaySphere KinematicsEngine FlightRecorder
        AtomicRingBufferOverwrite AsyncFlightRecorderStream
        SessionStateMachineStroboscopicTiming SerialPort)
    foreach(case ${GOLFSIM_TEST_CASES})
        add_test(NAME ${case} COMMAND GolfSimTests ${case})
    endforeach()
    add_test(NAME AllMathTests COMMAND GolfSimTests)

    include(tests/SeparationGuard.cmake)   # §3.3
endif()
```

Drift guard: the runner exits non-zero for an unknown case name, so a renamed test without a matching `GOLFSIM_TEST_CASES` edit fails `ctest` instead of silently vanishing.

### Step 2 — Test support headers

**`tests/TestAssert.hpp`** — as specified in 07 §2.1, extended:

```cpp
#pragma once
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <spdlog/spdlog.h>

struct TestFailure : std::runtime_error { using std::runtime_error::runtime_error; };

#define TEST_ASSERT_MSG(cond, msg)                                                   \
    do { if (!(cond)) {                                                              \
        spdlog::error("[TEST FAILURE] {}:{}  '{}'  ({})", __FILE__, __LINE__, #cond, msg); \
        throw TestFailure(std::string(#cond) + " @ " + __FILE__ + ":" + std::to_string(__LINE__)); \
    } } while (false)

#define TEST_ASSERT(cond) TEST_ASSERT_MSG(cond, "")

// For the ~30 `std::abs(a - b) < tol` checks: reports the actual values.
#define TEST_NEAR(a, b, tol)                                                         \
    do { const double _a = (a), _b = (b), _t = (tol);                                \
         if (!(std::abs(_a - _b) < _t)) {                                            \
             std::ostringstream _os; _os << #a " = " << _a << ", " #b " = " << _b << ", tol " << _t; \
             TEST_ASSERT_MSG(false, _os.str());                                      \
         } } while (false)
```

Not compiled away by `-DNDEBUG` because it never references `NDEBUG`/`assert`.

**`tests/TestRegistry.hpp`** — Meyers-singleton registry so static-init order across TUs is irrelevant:

```cpp
#pragma once
#include <functional>
#include <string>
#include <vector>

struct TestCase { std::string name; std::function<void()> fn; };
inline std::vector<TestCase>& testRegistry() { static std::vector<TestCase> r; return r; }
struct TestRegistrar { TestRegistrar(const char* n, void (*f)()) { testRegistry().push_back({n, f}); } };

// GOLFSIM_TEST(KinematicsEngine) { ... }   -> registers "KinematicsEngine"
#define GOLFSIM_TEST(name)                                                           \
    static void test_##name();                                                       \
    static TestRegistrar registrar_##name(#name, &test_##name);                      \
    static void test_##name()
```

**`tests/TestSandbox.hpp`** — one function: `TestSandbox::path(const std::string& leaf)` returns `GOLFSIM_TEST_SANDBOX_DIR / leaf` (the absolute path baked in by CMake above), creating the root if needed. Independent of cwd, so `ctest`, `./build/GolfSimTests` from the repo root, and an IDE run all land in `build/test_sandbox/`. Tests call `std::filesystem::remove_all` on their own leaf at start and end, as they do today.

**`tests/TestMain.cpp`** — behaviour:

| Invocation | Behaviour |
| :--- | :--- |
| `GolfSimTests` | run every registered case |
| `GolfSimTests Units KinematicsEngine` | run only those; **exit 2 if any name is unknown** |
| `GolfSimTests --filter Recorder` | substring match |
| `GolfSimTests --list` | print names, exit 0 |
| `--verbose` | spdlog level `info` (default `warn`, so test-internal `[TEST]` chatter is hidden unless asked) |

Each case runs inside `try { fn(); } catch (const std::exception& e) { … }`, prints `PASS name (12 ms)` / `FAIL name: <what>`, and the process exits 1 if any case failed. Summary line at the end.

### Step 3 — Move and convert `MathTests.cpp`

```bash
git mv src/MathTests.cpp tests/MathTests.cpp
sed -i 's/\bassert(/TEST_ASSERT(/g' tests/MathTests.cpp
```

Then by hand:
1. Replace `#include <cassert>` with `#include "TestAssert.hpp"`, `#include "TestRegistry.hpp"`, `#include "TestSandbox.hpp"`, `#include "Mocks.hpp"`.
2. Change each `void testUnits() {` → `GOLFSIM_TEST(Units) {` (11 edits; names must match `GOLFSIM_TEST_CASES` in Step 1).
3. Delete `runMathTests()` ([MathTests.cpp:924-943](../../src/MathTests.cpp#L924-L943)).
4. Optionally convert the `std::abs(x - y) < tol` asserts to `TEST_NEAR(x, y, tol)` — worth doing for the kinematics and triangulation cases where the numbers are the whole point; the rest can stay as `TEST_ASSERT`.

### Step 3b — Extract mocks to `tests/Mocks.hpp`

Cut [MathTests.cpp:704-755](../../src/MathTests.cpp#L704-L755) (`MockStrobeTrigger`, `MockStrobeVision`, `MockStrobeSpatial`, `MockStrobeKinematics`, `MockStrobeNet`) into `tests/Mocks.hpp` unchanged. They already use `static inline` members, so they are header-safe. §8 Phase C1/C4 will add `FakeUsbVideoDriver`, `FakeCameraNode`, `FakeCameraSystem` beside them.

### Step 4 — Sandbox all filesystem side effects

The only tests that hit real data paths are the three `SessionStateMachine` cases (they solve a mock shot → `recorder.saveSession` → `build/replays/` and `saveToShotHistory` → `build/shot_history.json`).

`SessionStateMachine` change — two trailing defaulted constructor parameters, source-compatible with `main.cpp`'s positional call:

```cpp
SessionStateMachine(Trigger t = Trigger(), …, PipelineTimingConfig timing = PipelineTimingConfig(),
                    std::string replayDir      = "build/replays",
                    std::string shotHistoryPath = "build/shot_history.json")
  : …, recorder(replayDir), shotHistoryPath_(std::move(shotHistoryPath)) { … }
```
`saveToShotHistory()` writes to `shotHistoryPath_` (drop the fallback to `./shot_history.json`; it only exists to paper over a missing `build/`).

Tests then construct with `TestSandbox::path("ssm_replays")` and `TestSandbox::path("ssm_history.json")`. `testFlightRecorder` / `testAsyncFlightRecorderStream` switch their hardcoded `build/replays_test` / `build/stream_test` to sandbox paths.

### Step 5 — Remove the recorder sleeps (−1.3 s)

`FlightRecorder`'s destructor already drains the queue before joining ([FlightRecorder.cpp:25-31](../../src/Diagnostics/FlightRecorder.cpp#L25-L31) — the worker exits only when `stopWorker && taskQueue.empty()`). So in both recorder tests, scope the recorder in a block and assert **after** it closes:

```cpp
{
    FlightRecorder recorder(dir);
    for (…) recorder.saveSession(frames, launchData);
}   // destructor: queue drained, files on disk, deterministic
// …directory assertions here…
```
Delete the `sleep_for(800ms)` and `sleep_for(500ms)`. Keep the 12 × 5 ms (folder names are millisecond-resolution timestamps) and the two 60 ms standby waits. Expected suite time ≈ 180–250 ms.

### Step 6 — `main.cpp`: remove all verification code from the start-up path

Delete [main.cpp:134-180](../../src/main.cpp#L134-L180) entirely — the `__cplusplus` probe, the banner, the four library smoke tests, `runMathTests()`, and `"Verification completed successfully!"`. Replace with a `--version` CLI flag handled in the existing arg loop:

```cpp
if (arg == "--version" || arg == "-v") {
  std::cout << "GolfSim  OpenCV " << CV_VERSION
            << "  Eigen " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION
            << "  spdlog " << SPDLOG_VER_MAJOR << "." << SPDLOG_VER_MINOR << "." << SPDLOG_VER_PATCH
            << "  json " << NLOHMANN_JSON_VERSION_MAJOR << "." << NLOHMANN_JSON_VERSION_MINOR << "." << NLOHMANN_JSON_VERSION_PATCH
            << "\n";
  return 0;
}
```
Net: `main()` goes straight from arg parsing to `"Starting Production Launch Monitor Pipeline"`. The `RUN_DEBUG_VIEWER` constant is a debugging toggle, not test code; refactor 06 owns it.

### Step 6b — Move the stray probe

`git mv test_xu.cpp scratch/test_xu.cpp`. It is Windows-only, not in any target, and its Linux counterpart (`scratch/dump_xu.py`) already lives there.

### Step 7 — Docs

- [README.md](README.md): row 07 → **COMPLETED**; roadmap node `Phase5` → Done styling.
- [BuildInstructions.md](../BuildInstructions.md): add "Running Tests" (§6 commands) and "Coverage" (§8.4 commands) for both platforms. Note that MSVC is multi-config, so `ctest` needs `-C Release`/`-C Debug`.
- `07_Test_Infrastructure.md` (the spec) stays as-is; §2 of this file records where implementation diverged.

---

## 6. Verification (for Steps 1–7)

Run in this order; each proves one goal from §1.

```bash
# Linux — Release is the important one (goal 2)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure          # expect 14/14 pass (11 cases + AllMathTests + 2 guards)
./build/GolfSimTests --list                          # expect exactly the 11 names in GOLFSIM_TEST_CASES
./build/GolfSimTests --filter Recorder               # expect 2 cases
time ./build/GolfSimTests                            # expect < 300 ms wall (goal 5)
./build/GolfSim --version                            # one line, exit 0
```

**Negative test (goal 2, mandatory):** temporarily change one `TEST_ASSERT` to an obviously false condition, rebuild Release, confirm `ctest` reports that single case FAILED and `AllMathTests` FAILED. Revert.

**Negative test (goal 7, mandatory):** temporarily add `// TEST_ASSERT(x)` to any file under `src/`, run `ctest -R NoTestCodeInProduction`, confirm it FAILS and names the line. Revert.

**Zero side effects (goals 1, 4):**
```bash
ls build/replays | sort > /tmp/before; stat -c %s build/shot_history.json
./build/GolfSimTests && ./build/GolfSim        # (no cameras → clean exit)
ls build/replays | sort | diff - /tmp/before   # expect no diff
stat -c %s build/shot_history.json             # expect same size
ls build/test_sandbox                          # expect only the sandbox leaves, cleaned or empty
```
Also: `./build/GolfSim` should reach `[System] Initializing camera drivers...` within ~50 ms of launch (today it's ≈ 1.5 s).

**Production binary has no test symbols:** `cmake -B build -DGOLFSIM_BUILD_TESTS=OFF && cmake --build build && nm -C build/GolfSim | grep -c TestRegistrar` → `0`.

**Windows:**
```powershell
& $cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

**Unknown-name guard:** `./build/GolfSimTests DoesNotExist` → exit code 2.

---

## 7. Risks and Gotchas

- **Latent failures may surface.** The suite has only ever been *validated* in Debug. The first Release run with real assertions may expose a case that was silently passing. Treat those as findings about the code under test, not as regressions of this refactor; fix or `TEST_ASSERT_MSG(false, "known: …")` them explicitly, never delete.
- **`GLOB` + new files:** adding a file under `tests/` requires re-running configure (already noted in BuildInstructions).
- **`SessionStateMachine` is a class template with a defaulted-argument constructor.** Adding trailing defaulted parameters is safe for the four existing instantiations (main.cpp + 3 tests) because all pass positional args and none pass beyond `timing`.
- **80 MB per SSM instance** (40-frame preallocated pool). The three SSM tests construct three instances sequentially; fine, but don't parallelise them within one process.
- **Registrar and MSVC:** `static TestRegistrar registrar_X` at namespace scope in a `.cpp` is fine; do not move `GOLFSIM_TEST` bodies into a header.
- **`saveToShotHistory` fallback removal** changes behaviour only if `build/` doesn't exist at runtime — which already breaks `session.log` and replays, so it's not a real path.
- **Removing the environment banner** changes the console output people may be used to seeing on launch. `--version` preserves the information; nothing else reads that output.
- **Separation guard is a grep.** It will flag a comment that mentions `MockStrobeTrigger`. That is intended — comments in production code should not reference test types either.

**Effort (Steps 1–7):** ~9 files touched, ~220 new lines, ~90 edited/deleted lines; one sitting.

---

## 8. Coverage Plan: 100 % Line / Statement / Logic

### 8.1 Definitions and tool choice

| Metric requested | Precise definition used here | Tool |
| :--- | :--- | :--- |
| **Line** | Every executable source line executed ≥ 1 time | `llvm-cov` (Lines) and `gcov` (cross-check) |
| **Statement** | Every *code region* executed ≥ 1 time. `gcov` cannot see two statements on one line; `llvm-cov`'s source-based regions can | `llvm-cov` (Regions) |
| **Logic** | (a) **Branch**: every conditional edge taken both ways; (b) **MC/DC**: every boolean operand in a compound condition shown to independently affect the outcome | `llvm-cov` (Branches, MC/DC via `-fcoverage-mcdc`); `gcov --conditions` cross-check |

**Primary tool: Clang 22 + `llvm-cov`** (both installed, no packages needed). It is the only installed tool that reports all three metrics from one build. Verified today: the heaviest translation unit compiles cleanly under `clang++ -fprofile-instr-generate -fcoverage-mapping -fcoverage-mcdc`.

**Cross-check: GCC 16 `gcov`** with `-fcondition-coverage`, reported through `gcovr 8.6` (`sudo pacman -S gcovr`, one-time). Same compiler as the dev build; catches any Clang/GCC divergence. Run at phase boundaries, not every iteration.

Both toolchains exclude **exception-unwind edges** from branch totals (`llvm-cov` does not count them; `gcovr --exclude-throw-branches`). Today's raw `gcov` output counts them, which is why the naive branch total is 9 002 vs. 6 271 real branches.

### 8.2 Measured baseline (2026-09-19, GCC gcov, throw-branches excluded, one `./GolfSim` run = current test suite + no-camera start-up)

| Layer | Lines | Branches | MC/DC |
| :--- | :---: | :---: | :---: |
| **Production** (`src/` + `include/`, excl. `main.cpp`) | **1438 / 2361 = 60.9 %** | **1884 / 4232 = 44.5 %** | **697 / 2098 = 33.2 %** |
| — pure logic (`Math/`, `Orchestration/`, `Diagnostics/`) | 1342 / 1804 = 74.4 % | 1831 / 3788 = 48.3 % | 668 / 1808 = 36.9 % |
| — HAL / Camera / ThreadManager | 96 / 557 = 17.2 % | 53 / 444 = 11.9 % | 29 / 290 = 10.0 % |
| `src/main.cpp` (app layer) | 68 / 451 = 15.1 % | 104 / 930 = 11.2 % | 13 / 320 = 4.1 % |

Per file, production only (files with 0 executable lines omitted):

| File | Lines | Branches | MC/DC | Never-executed functions |
| :--- | :---: | :---: | :---: | :--- |
| `Math/StereoTriangulator.cpp` | 86.8 % | 81.3 % | 74.5 % | default ctor, `setCalibration` |
| `Math/StereoBallTrackerTrigger.cpp` | 82.9 % | 43.8 % | 37.8 % | `isStandbyRequested`, `getLatestDiagnostics` |
| `Diagnostics/FlightRecorder.cpp` | 82.7 % | 67.3 % | 44.6 % | — (error paths only) |
| `Math/BallPresenceTrigger.cpp` | 77.9 % | 37.4 % | 32.0 % | — (branches) |
| `Orchestration/SessionStateMachine.hpp` | 68.6 % | 30.8 % | 25.4 % | `ConcreteSSM` instantiation entirely: `sendSerialCommand`, `saveToShotHistory`, `setStreamRecordingMode`, `processNextFrame` stream branch |
| `Math/EigenBallisticsEngine.cpp` | 67.7 % | 57.5 % | 43.1 % | — (branches) |
| `Math/OpenCVMomentsTracker.cpp` | 60.7 % | 38.9 % | 28.0 % | — (overlap / Hough branches) |
| `Math/AtomicRingBuffer.hpp` | 94.0 % | 50.0 % | 50.0 % | `push`, `pop`, `preallocate` (only `push_overwrite`/`pop_into` tested) |
| `Camera/FrameSet.hpp` | 100 % | 37.5 % | 40.0 % | `swap` |
| `HAL/SerialPort.cpp` | 41.9 % | 45.9 % | 50.0 % | `getPosixBaud` (success path never runs) |
| `HAL/V4L2Driver.cpp` | 12.9 % | 9.5 % | 7.4 % | everything past enumeration |
| `Math/TcpJsonTransmitter.cpp` | 0 % | 0 % | 0 % | all |
| `Orchestration/ThreadManager.cpp` | 0 % | 0 % | 0 % | all |
| `Camera/HardwareSyncedCameraSystem.cpp` | 0 % | 0 % | 0 % | all |
| `Camera/OV9281CameraNode.cpp` | 0 % | 0 % | 0 % | all |
| `Diagnostics/GlobalLogger.cpp` | 0 % | 0 % | 0 % | all |
| `Camera/PlaybackCameraNode.cpp` | 0 % | — | — | stub (06) |

### 8.3 Scope and the denominator

**Gated scope:** every `.cpp`/`.hpp` under `src/` and `include/`, measured on Linux.

**What is *not* in the denominator, and why**

| Excluded | Mechanism | Justification |
| :--- | :--- | :--- |
| Windows-only code (`MediaFoundationDriver`, `Win32Serial`, `#ifdef _WIN32` blocks) | Not compiled on Linux → invisible to the instrumenter. **Covered separately** by a Windows coverage run (§8.5 Phase C6) | Cannot execute on the gating platform |
| `tests/`, `tools/`, `scratch/`, `firmware/` | Path filter | Not production C++; see §8.6 for the optional plan |
| Lines marked `// LCOV_EXCL_LINE` / `LCOV_EXCL_START…STOP` | Explicit, per-line, **with a reason on the same line**. `llvm-cov` itself ignores these comments, so `check_thresholds.py` applies them to the LCOV export (`DA`/`BRDA` records) for lines and branches; regions and MC/DC come from the JSON summary unmodified | See policy below |

**Exclusion policy** (reviewed like code; the report prints the excluded-line count and the gate fails if it exceeds **2 % of production lines**):

1. OS failure paths that are *pure logging and return*, provable unreachable without fault injection, **and** not worth a seam — e.g. `fs::create_directories` throwing inside `FlightRecorder`'s constructor. Prefer a seam when the branch carries logic.
2. `default:` arms of `switch` over an exhaustive `enum class`.
3. Defensive `if (!ptr) return;` on pointers the type system already guarantees — better fixed by removing the check.

Everything else is reachable and must be reached.

### 8.4 Tooling steps (Phase C0)

1. **CMake option** `GOLFSIM_COVERAGE` (default OFF). When ON with Clang: adds `-fprofile-instr-generate -fcoverage-mapping -fcoverage-mcdc -O0 -g` to `golfsim_core` and `GolfSimTests`; with GCC: `--coverage -fcondition-coverage -O0 -g`. Refuses (`message(FATAL_ERROR)`) if `CMAKE_BUILD_TYPE` is not Debug — optimised code folds branches and lies. **`GolfSimTests` links `golfsim_core` with `$<LINK_LIBRARY:WHOLE_ARCHIVE,…>` in this mode** — otherwise the linker drops every object no test references and those files silently vanish from the denominator (first run reported 78.5 % lines because the seven 0 % files were missing).
2. **Custom target `coverage`** (`cmake --build build-cov --target coverage`):
   ```
   ctest                                   # LLVM_PROFILE_FILE=%p.profraw per test process
   llvm-profdata merge -o merged.profdata *.profraw
   llvm-cov report GolfSimTests -instr-profile=merged.profdata \
       -ignore-filename-regex='(tests|/usr)/' -show-mcdc-summary -show-branch-summary
   llvm-cov show   … -format=html -output-dir=coverage-html   # for reading
   llvm-cov export … -format=text > coverage.json             # for the gate
   ```
3. **Gate script** `tools/coverage/check_thresholds.py`: reads `coverage.json` and `tools/coverage/thresholds.json` (`{"lines": 60.9, "regions": …, "branches": 44.5, "mcdc": 33.2}`), fails if any metric is **below** its threshold, and prints the per-file table sorted by gap. Registered as CTest case `CoverageGate` (only when `GOLFSIM_COVERAGE=ON`).
4. **Ratchet rule:** every PR that adds tests raises `thresholds.json` to the new measured value. Thresholds only go up. `100.0` is the terminal state for each metric.
5. **Cross-check target** `coverage-gcc` using a second build dir with GCC + `gcovr --exclude-throw-branches --html-details --json`. Compared at phase boundaries; a > 1 % disagreement on any file is investigated (usually a macro or template instantiation counted differently).
6. **Recommended invocation** (`CMakePresets.json` is gitignored → document in BuildInstructions instead):
   ```bash
   cmake -B build-cov -S . -G Ninja -DCMAKE_BUILD_TYPE=Debug -DGOLFSIM_COVERAGE=ON \
         -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
   cmake --build build-cov --target coverage
   xdg-open build-cov/coverage-html/index.html
   ```

### 8.5 Phases — what to test, and the seam each needs

Each phase ends with the ratchet raised and the per-file table in the PR description. Phases C1–C4 need nothing from refactor 06; C5 does.

#### Phase C1 — Pure logic to 100 % (≈ 1 800 lines; from 74 % → 100 %)

No new seams; these are gaps in existing unit tests. Per module, the branches the baseline shows untaken:

| Module | Untested today → cases to add |
| :--- | :--- |
| `Units.hpp`, `LaunchData.hpp`, `PipelineTimingConfig.hpp` | Every conversion direction; `isValidTiming()` false paths (each field individually invalid) |
| `AtomicRingBuffer.hpp` | `push` (blocking variant) full/not-full; `pop` empty/not-empty; `preallocate`; wrap-around at capacity; two-thread producer/consumer smoke with `std::jthread` (bounded iterations, no sleeps) |
| `FrameSet.hpp` | `swap` (member + ADL), `copyTo` with an empty source frame (the `if (!frames[i].empty())` branch) |
| `EigenBallisticsEngine.cpp` | < 3 points; collinear points; zero-velocity; negative VLA; each marker-confidence branch; spin axis when markers are absent |
| `StereoTriangulator.cpp` | Default ctor + `setCalibration`; rays that don't intersect (skew, parallel); left-only / right-only / neither; epipolar rejection; the 2×2 covariance sort with 2 points, 1 point, 0 points, and degenerate identical points |
| `OpenCVMomentsTracker.cpp` | Overlapping-contour path (`area > 1.5·min && circularity < min`) with and without Hough hits; marker threshold with zero markers; contour on ROI edge; empty frame; wrong-type frame |
| `BallPresenceTrigger.cpp` | Every state transition arm with a synthetic frame sequence, including re-arm after `BALL_DEPARTED`, template mismatch below threshold, and standby → wake |
| `StereoBallTrackerTrigger.cpp` | `getLatestDiagnostics()` (never called!), `isStandbyRequested()`, `CAPTURED` → `reset()`, grace-frame exhaustion, nudge beyond tolerance, epipolar mismatch between L/R candidates |
| `SessionStateMachine.hpp` | Stream mode (`setStreamRecordingMode` → chunk flush), `sendSerialCommand` de-dup (same command twice → one callback), standby request → `'L'`, empty left/right frame early return, `pointBufferFull` completion path, insufficient points → discard, **and one instantiation with the real `ConcreteSSM` types** so the production template is covered (feed synthetic frames with a drawn ball) |
| `FlightRecorder.cpp` | `enforceLimit` with mixed `shot_`/`stream_` names; metadata write failure (make `replayPath/metadata.json` a directory first); every annotation branch (`accepted`, rejected, overlapping, markers present/absent, 3-D coords present/absent); stream task with empty frames |
| `GlobalLogger.cpp` | `getInstance` idempotence, `setLevel` each level |

Deliverable: pure-logic layer at 100/100/100. Expected: ~45 new cases, split into per-module files (`tests/TriggerTests.cpp` etc.) as they are written.

#### Phase C2 — I/O with real OS resources, no hardware (≈ 260 lines)

| Module | Technique |
| :--- | :--- |
| `TcpJsonTransmitter.cpp` | Test spins up a loopback listener (`socket/bind/listen` on port 0, read the port back) in a `std::jthread`; exercises connect-success, connect-refused, send-after-disconnect, partial-write handling, destructor-closes-socket, and JSON payload shape. POSIX sockets on Linux; the same test compiles on Windows with the existing `ws2_32` link |
| `SerialPort.cpp` (POSIX half) | `openpty()` gives a real tty pair without hardware: open the slave path, verify termios settings via `tcgetattr`, `writeChar`/`writeString`/`flush`, read the bytes back from the master fd. Covers `getPosixBaud` for every case incl. default. Keep the existing non-existent-port retry test |

Seams needed: none.

#### Phase C3 — HAL drivers (≈ 720 lines; V4L2 from 13 % → 100 %)

`V4L2Driver` needs two things because no real device can provoke `mmap` failure, `ENOSPC`, `EINTR`, or a driver substituting the pixel format:

1. **`V4L2SysCalls` seam** — a struct of function pointers (`open, close, ioctl, mmap, munmap, poll`) with defaults bound to the libc symbols, passed to the constructor with a defaulted argument. Production behaviour is unchanged; the fake in `tests/FakeV4L2.hpp` scripts responses per ioctl request (`VIDIOC_ENUM_FMT` → format list, `VIDIOC_S_FMT` → echo/adjust, `VIDIOC_DQBUF` → next canned buffer with a chosen timestamp, `STREAMON` → `ENOSPC`, …). This reaches every branch: three pixel formats, size mismatch warning, no-format error, `REQBUFS` < 2, `mmap` fail, `POLLERR`, short buffer, `V4L2_BUF_FLAG_ERROR`, exposure clamp at min/max, `EINTR` retry in `xioctl`.
2. **`vivid` integration test** — CTest label `hardware`, skipped unless a `vivid` node exists (`sudo modprobe vivid n_devs=1`). Runs the real ioctl path end-to-end: enumerate → initialise → 10 grabs → timestamps monotonic → shutdown from a second thread while a grab is blocked (the handshake in 08 §6). Not in the coverage gate (needs root once) but run before every hardware session.

Deliverable: HAL layer 100 % on Linux with the fake; the integration test proves the fake matches reality.

#### Phase C4 — Camera nodes and ThreadManager (≈ 170 lines; 0 % → 100 %)

| Module | Technique |
| :--- | :--- |
| `OV9281CameraNode.cpp` | `FakeUsbVideoDriver` (records calls, returns scripted frames/timestamps) — covers `captureFrame` true/false, null-driver paths, `setExposure`, `getLastFrameTimestampUs`, `shutdown` |
| `HardwareSyncedCameraSystem.cpp` | `FakeCameraNode` × 2 — no cameras, one failing camera, both succeed, timestamp propagation, `shutdown` fan-out |
| `ThreadManager.cpp` | `FakeCameraSystem` that emits N synthetic frames then reports failure, real `AtomicRingBuffer`, real `ConcreteSSM` with sandbox paths. Covers start-twice warning, consumer-before-producer warning, stop-twice, stop-while-blocked, destructor-stops. Bounded by frame count, no sleeps except the loops' own 1 ms yields |

Seams needed: none (all constructor-injected already).

#### Phase C5 — Application layer, `main.cpp` (451 lines; 15 % → 100 %) — **depends on refactor 06**

`main.cpp` is untestable in-process today because `runCameraDebugViewer` and `runReplayViewer` call `cv::imshow`/`cv::waitKey` directly inside their loops, and `main()` does arg parsing, wiring and a blocking `std::cin.get()`. Refactor 06 already plans the extractions; this phase adds the seam that makes them coverable:

1. `AppConfig parseArgs(int argc, char** argv)` in `src/App/AppConfig.cpp` → pure function, unit-tested for every flag, every default, `--swap`, malformed values.
2. `ReplayViewer` and `CameraDebugger` take an **`IDisplay`** (`show(name, mat)`, `int waitKey(ms)`, `destroy()`) — `OpenCvDisplay` in production, `ScriptedDisplay` in tests that returns a key sequence (`' '`, `'d'`, `'o'`, `27`) and captures the composites. Every key branch, the wrap-around indices, the missing-frame break, and the overlays-off conversion become unit-testable against a fixture replay under `tests/fixtures/shot_fixture/`.
3. `main()` shrinks to ≈ 30 lines: `parseArgs` → dispatch → `runLaunchMonitor(config, std::istream& shutdownSignal)`. The remaining lines are covered by **subprocess CTest cases**: `GolfSim --version`, `GolfSim` with no cameras (clean exit), `GolfSim --replay <fixture>` with `ScriptedDisplay` selected by an env var, and `GolfSim --stream` against a `PlaybackCameraNode` (06 item 4) with `echo | GolfSim` supplying the shutdown newline.

Until 06 lands, `main.cpp` is **measured and reported but not gated**; the gate's `thresholds.json` carries a `"gated_paths"` list that gains `src/main.cpp` when C5 starts.

#### Phase C6 — Windows-only code (`MediaFoundationDriver`, `Win32Serial`, `_WIN32` blocks)

Cannot be measured on Linux. Plan: a **Windows coverage run with clang-cl + `llvm-cov`** (Clang ships with VS 2022) using the same `GOLFSIM_COVERAGE` option, driven by an integration test against the real camera plus a `FakeMediaSource` only if the branch count justifies it. Tracked as a separate gate (`thresholds-windows.json`) ratcheted on the Windows machine.

This is the one place where "100 % of the repo" is honest only as "100 % of what is compilable per platform, gated per platform".

### 8.6 Optional scope extensions (not gated unless you say so)

| Code | Approach | Effort |
| :--- | :--- | :--- |
| `firmware/strobe_controller.ino` (safety-relevant: 50 µs clamp, 10 s watchdog) | Host-side build: `tests/firmware/ArduinoShim.hpp` fakes `millis()`, `delayMicroseconds()`, `digitalWrite`, `Serial`, `TCCR1A/OCR1A`; compile the `.ino` as C++ into a `StrobeFirmwareTests` target. MC/DC on the watchdog and mode conditions becomes measurable with the same `llvm-cov` flow | ~1 day |
| `tools/analysis/*.py`, `tools/debug_ir_strobe.py` | `pytest --cov=tools` with a fixture replay; the analysis script's functions are already pure enough | ~½ day |
| `scratch/` | Never |

### 8.7 Order and dependencies

```
Steps 1–7 (this doc)  →  C0 tooling + baseline gate
                         ├→ C1 pure logic        (no deps)      ┐
                         ├→ C2 sockets / pty     (no deps)      ├→ production layer 100 %
                         ├→ C3 V4L2 seam + vivid (no deps)      │
                         └→ C4 nodes + threads   (no deps)      ┘
Refactor 06 ──────────→  C5 app layer (parseArgs, IDisplay, PlaybackCameraNode)
Windows machine ──────→  C6 MF / Win32Serial (separate gate)
```

C1–C4 are independent and can be interleaved with hardware testing sessions — each is a self-contained PR that raises the ratchet.

### 8.8 Honest constraints

- **MC/DC on OpenCV/Eigen-heavy code** counts operands inside library macros and inlined templates as ours when they expand in our translation units. `-ignore-filename-regex='/usr/'` removes header-only library code from the report; anything remaining is genuinely in our source.
- **Coverage ≠ correctness.** 100 % says every line and decision was *exercised*, not that the assertions were *meaningful*. Phase C1's cases must assert outcomes, not merely call functions. Review for "calls X" tests that assert nothing.
- **Two timing-dependent tests** (60 ms standby) stay wall-clock; they are already deterministic in practice. An injectable clock is the fix if they ever flake.
- **Exclusion budget is 2 %.** If a module cannot reach 100 % without exceeding it, the answer is a seam, not more markers.

---

## 9. Implementation Record (2026-09-19)

Steps 1–7 and Phase C0 were implemented in one pass; everything in §6 was executed.

| Check (§6) | Result |
| :--- | :--- |
| Debug `ctest` | 14/14 pass (11 cases, `AllMathTests`, 2 separation guards) |
| Release `ctest` (`-O3 -DNDEBUG`) | 14/14 pass |
| Negative: one assertion broken, Release | `Units` **FAILED**, `AllMathTests` **FAILED** — assertions survive `NDEBUG` |
| Negative: `// TEST_ASSERT(x)` added to `src/Camera/FrameSet.cpp` | `NoTestCodeInProduction` **FAILED** naming the file and line |
| `-DGOLFSIM_BUILD_TESTS=OFF` | no `GolfSimTests` target; `nm -C GolfSim` has 0 `TestRegistrar` / `test_*` symbols |
| Zero side effects | `build/replays/` listing and `build/shot_history.json` size unchanged after running the suite and `GolfSim`; sandbox holds only `ssm_replays/`, `ssm_shot_history.json` |
| Start-up to `Initializing camera drivers` | **82 ms** (was ≈ 1.5 s) |
| `GolfSimTests DoesNotExist` | exit 2 |
| `GolfSim --version` | one line, exit 0 |
| Suite wall time, Release | **371 ms** — above the 300 ms goal. 220 ms is the deliberate wall-clock waits kept by Step 5 (2 × 60 ms standby timeouts, 12 × 5 ms folder-name spacing, 2 × 20 ms serial retries); the rest is real PNG encoding of 1280 × 800 frames in the SSM test. Acceptable; revisit with an injectable clock if it ever matters |
| Coverage target (Clang 22, `llvm-cov`) | builds, runs, reports, gates; `CoverageGate` passes at baseline and fails when a threshold is raised above the measured value |

**Phase C0 baseline (llvm-cov, gated scope = `src/` + `include/` minus `main.cpp`)**

| Metric | Baseline | Threshold set |
| :--- | :---: | :---: |
| Lines | 58.94 % | 58.9 |
| Regions (statements) | 41.85 % | 41.8 |
| Branches | 42.56 % | 42.5 |
| MC/DC | 13.33 % | 13.3 |

These differ from the §8.2 gcov numbers because the two tools define regions/branches/conditions differently (llvm-cov MC/DC requires independence pairs; gcov's condition coverage counts each outcome). The ratchet is against llvm-cov from here on; gcov remains the cross-check.

**Deviations from the plan discovered while implementing**
- Whole-archive linking is required in coverage mode (§8.4 item 1, added).
- `llvm-cov` does not honour `LCOV_EXCL_*` comments; the gate script applies them itself from the LCOV export (§8.3, corrected).
- `add_custom_target(coverage …)` needs `VERBATIM` or the `-E "NoTest|Coverage"` regex is split by the shell.

**Next:** Phase C1 (pure logic to 100 %) — start with `SessionStateMachine` stream mode and the never-called `StereoBallTrackerTrigger::getLatestDiagnostics()`, per the §8.5 table.
