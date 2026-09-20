cmake_minimum_required(VERSION 3.20)
project(GolfSim LANGUAGES CXX)

# Configure C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(GOLFSIM_BUILD_TESTS "Build the GolfSimTests runner and register it with CTest" ON)
option(GOLFSIM_COVERAGE   "Instrument golfsim_core and GolfSimTests for coverage (Debug only)" OFF)

find_package(OpenCV 5.0 REQUIRED COMPONENTS
    core
    imgproc
    geometry
    stereo
    calib
    imgcodecs
    highgui
    videoio
)
set(GOLFSIM_OPENCV_LIBS ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)

# -----------------------------------------------------------------------------
# Core library: every production source except the entry point.
# Both the application and the test runner link this, so nothing is compiled
# twice and the source list can never drift between the two targets.
# -----------------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES "src/*.cpp")
list(FILTER CORE_SOURCES EXCLUDE REGEX "src/main\\.cpp$")
add_library(golfsim_core STATIC ${CORE_SOURCES})

target_include_directories(golfsim_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(golfsim_core PUBLIC
    ${GOLFSIM_OPENCV_LIBS}
    Eigen3::Eigen
    spdlog::spdlog
    nlohmann_json::nlohmann_json
)

# OS-Specific API configuration
if(WIN32)
    # <windows.h> (via Media Foundation / Winsock) defines min/max macros that
    # break std::numeric_limits<T>::max() and friends.
    target_compile_definitions(golfsim_core PUBLIC NOMINMAX)
    # Link Windows Media Foundation and Winsock libraries
    target_link_libraries(golfsim_core PUBLIC
        mfplat
        mfreadwrite
        mfuuid
        mf
        ksuser
        ole32
        strmiids
        ws2_32
    )
endif()

# -----------------------------------------------------------------------------
# Production binary
# -----------------------------------------------------------------------------
add_executable(GolfSim src/main.cpp)
target_link_libraries(GolfSim PRIVATE golfsim_core)

# -----------------------------------------------------------------------------
# Camera delivery benchmark (tools/bench/CameraBench.cpp): measures the raw
# driver frame rate with nothing downstream. Build Release before trusting it.
# -----------------------------------------------------------------------------
add_executable(CameraBench tools/bench/CameraBench.cpp)
target_link_libraries(CameraBench PRIVATE golfsim_core)

# -----------------------------------------------------------------------------
# Coverage instrumentation (Phase C0 of docs/refactor/07_Test_Infrastructure_Plan.md)
# Clang: source-based coverage with regions, branches and MC/DC -> llvm-cov.
# GCC:   gcov with condition (MC/DC) coverage -> gcovr, used as a cross-check.
# -----------------------------------------------------------------------------
if(GOLFSIM_COVERAGE)
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(FATAL_ERROR "GOLFSIM_COVERAGE requires -DCMAKE_BUILD_TYPE=Debug; optimised builds fold branches and misreport coverage.")
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(GOLFSIM_COVERAGE_FLAGS -fprofile-instr-generate -fcoverage-mapping -fcoverage-mcdc -O0 -g)
        set(GOLFSIM_COVERAGE_LINK  -fprofile-instr-generate)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(GOLFSIM_COVERAGE_FLAGS --coverage -fcondition-coverage -O0 -g)
        set(GOLFSIM_COVERAGE_LINK  --coverage)
    else()
        message(FATAL_ERROR "GOLFSIM_COVERAGE is supported with Clang or GCC only (got ${CMAKE_CXX_COMPILER_ID}).")
    endif()
    target_compile_options(golfsim_core PUBLIC ${GOLFSIM_COVERAGE_FLAGS})
    target_link_options(golfsim_core PUBLIC ${GOLFSIM_COVERAGE_LINK})
    message(STATUS "Coverage instrumentation enabled (${CMAKE_CXX_COMPILER_ID})")
endif()

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(GOLFSIM_BUILD_TESTS)
    enable_testing()

    file(GLOB TEST_SOURCES "tests/*.cpp")
    add_executable(GolfSimTests ${TEST_SOURCES})
    if(GOLFSIM_COVERAGE)
        # Pull every object out of the static library so untested files still
        # appear in the coverage denominator at 0% instead of vanishing.
        target_link_libraries(GolfSimTests PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,golfsim_core>")
    else()
        target_link_libraries(GolfSimTests PRIVATE golfsim_core)
    endif()

    # Absolute sandbox root so the runner behaves the same from any cwd
    # (ctest, IDE, shell). Every filesystem side effect of a test lands here.
    target_compile_definitions(GolfSimTests PRIVATE
        GOLFSIM_TEST_SANDBOX_DIR="${CMAKE_BINARY_DIR}/test_sandbox"
    )

    # One CTest entry per case. The runner exits 2 for an unknown name, so a
    # renamed test that is not updated here fails ctest instead of vanishing.
    set(GOLFSIM_TEST_CASES
        Units
        BallPresenceTrigger
        StereoBallTrackerTrigger
        OpenCVMomentsTracker
        StereoTriangulatorAndRaySphere
        KinematicsEngine
        FlightRecorder
        AtomicRingBufferOverwrite
        AsyncFlightRecorderStream
        SessionStateMachineStroboscopicTiming
        SerialPort
        CameraConfig
        AppConfig
        DotClusterFinder
        DotClusterTracker
    )
    foreach(case ${GOLFSIM_TEST_CASES})
        add_test(NAME ${case} COMMAND GolfSimTests ${case})
    endforeach()
    add_test(NAME AllMathTests COMMAND GolfSimTests)

    include(tests/SeparationGuard.cmake)

    # `cmake --build build-cov --target coverage` — run the suite under
    # instrumentation and produce the report + gate (Clang/llvm-cov path).
    if(GOLFSIM_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        find_program(LLVM_PROFDATA llvm-profdata REQUIRED)
        find_program(LLVM_COV llvm-cov REQUIRED)
        add_custom_target(coverage
            COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/profraw ${CMAKE_BINARY_DIR}/coverage-html
            COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/profraw
            COMMAND ${CMAKE_COMMAND} -E env LLVM_PROFILE_FILE=${CMAKE_BINARY_DIR}/profraw/%p.profraw
                    ${CMAKE_CTEST_COMMAND} --test-dir ${CMAKE_BINARY_DIR} -E "NoTest|Coverage" --output-on-failure
            COMMAND ${CMAKE_COMMAND} -DPROFRAW_DIR=${CMAKE_BINARY_DIR}/profraw
                    -DLLVM_PROFDATA=${LLVM_PROFDATA} -DLLVM_COV=${LLVM_COV}
                    -DBINARY=$<TARGET_FILE:GolfSimTests> -DOUT_DIR=${CMAKE_BINARY_DIR}
                    -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                    -P ${CMAKE_CURRENT_SOURCE_DIR}/tools/coverage/llvm_cov_report.cmake
            COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/coverage/check_thresholds.py
                    ${CMAKE_BINARY_DIR}/coverage.json ${CMAKE_CURRENT_SOURCE_DIR}/tools/coverage/thresholds.json
                    --source-dir ${CMAKE_CURRENT_SOURCE_DIR}
            DEPENDS GolfSimTests GolfSim
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Running tests under coverage instrumentation and checking thresholds"
            USES_TERMINAL
            VERBATIM
        )
        # Re-runnable gate over the last `coverage` target output.
        add_test(NAME CoverageGate
            COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tools/coverage/check_thresholds.py
                    ${CMAKE_BINARY_DIR}/coverage.json ${CMAKE_CURRENT_SOURCE_DIR}/tools/coverage/thresholds.json
                    --source-dir ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
endif()
