# Refactor 07: Testing Infrastructure & CTest Integration

This document specifies the decoupled test architecture, CTest integration, and release-safe test assertions for the GolfSim codebase.

---

## 1. Problem Statement

### 1.1 Tests Executed Synchronously on Application Startup
In `src/main.cpp`:
```cpp
// Line 145:
runMathTests(); // Run verification tests on startup
```
Every time the user runs `./build/GolfSim` (even in a live simulation or debug session), the application pauses to execute the full math test suite. `testFlightRecorder()` writes 12 test sessions to disk and sleeps for $800\text{ ms}$, creating an unavoidable 1-second startup delay and writing temporary files into `build/replays_test/`.

### 1.2 The Release Build Assert Disabler (`-DNDEBUG`)
All unit tests in `src/MathTests.cpp` use standard C library `assert()`:
```cpp
assert(std::abs(mph.value() - 22.36936) < 1e-4);
```
In standard C and C++, when building in **Release** mode (`CMAKE_BUILD_TYPE=Release`), the compiler automatically defines the `-DNDEBUG` preprocessor macro.
Under `-DNDEBUG`, standard `assert(expr)` compiles into an **empty no-op**!
As a result, in production Release builds, the entire test suite executes without actually validating a single condition!

---

## 2. Decoupled Test Architecture

```
                                [ CMake Build System ]
                                          │
                     ┌────────────────────┴────────────────────┐
                     ▼                                         ▼
            [ Target: GolfSim ]                      [ Target: GolfSimTests ]
            • Production binary                      • Standalone test runner
            • Fast startup (<5ms)                    • Integrated with CTest
            • Zero test overhead                     • Release & Debug validated
```

### 2.1 Release-Safe Test Assertion Macro
We will implement a lightweight, release-safe assertion macro `TEST_ASSERT(expr, msg)` that cannot be compiled away by `-DNDEBUG`:

```cpp
#include <stdexcept>
#include <string>
#include <spdlog/spdlog.h>

#define TEST_ASSERT(condition, msg) \
    do { \
        if (!(condition)) { \
            spdlog::error("[TEST FAILURE] {}:{} - Assertion failed: '{}' ({})", \
                          __FILE__, __LINE__, #condition, msg); \
            throw std::runtime_error(std::string("Assertion failed: ") + #condition); \
        } \
    } while (false)
```

### 2.2 CMakeLists.txt & CTest Integration
Add testing targets to `CMakeLists.txt`:

```cmake
enable_testing()

# Standalone test runner
add_executable(GolfSimTests
    tests/TestMain.cpp
    src/MathTests.cpp
    src/Math/BallPresenceTrigger.cpp
    src/Math/StereoBallTrackerTrigger.cpp
    src/Math/OpenCVMomentsTracker.cpp
    src/Math/StereoTriangulator.cpp
    src/Math/EigenBallisticsEngine.cpp
    src/Diagnostics/FlightRecorder.cpp
)

target_link_libraries(GolfSimTests PRIVATE
    ${GOLFSIM_OPENCV_LIBS}
    Eigen3::Eigen
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

target_include_directories(GolfSimTests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(NAME AllMathTests COMMAND GolfSimTests)
```

### 2.3 Automated Testing Workflow
Developers and CI pipelines can run all tests cleanly via:
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --target GolfSimTests
ctest --test-dir build --output-on-failure
```
Production users running `./build/GolfSim` experience instantaneous startup with zero test delays.

