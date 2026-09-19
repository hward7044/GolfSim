# GolfSim Build Instructions for AI Agents

To avoid compiler or header resolution errors (e.g., standard library headers like `<iostream>` or `<limits>` not being found), follow these build instructions.

## The Environment Issue
This project uses Windows MSVC. Running standard `cmake` from a default shell will fail to locate compiler tools and standard headers because the path is not configured.

## Option 1: Use the VS Code Task (Recommended)
VS Code is configured with a build task that runs in the proper environment. If you want to instruct the user or run it through workspace commands, use the VS Code task:
- Task Label: `Build GolfSim`

## Option 2: Dev PowerShell or Command Prompt Paths
If you need to compile from the terminal, locate the Visual Studio compiler environment. On this system:
- **CMake Executable:** `C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`
- **Ninja Executable:** `C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe`

### Run configured CMake preset
Always use the default preset configuration (`default`) which sets up the vcpkg toolchain path correctly:

```powershell
$cmake = "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
& $cmake --preset default
& $cmake --build build --config Debug
```

*Note: If files are added or deleted, make sure to re-run the configure preset (`--preset default`) before building.*

---

## Linux Environment (Arch / Omarchy)

For detailed information on installed tools, compilers, and library versions, see [ToolsAndLibraries.md](file:///home/hward/Projects/GolfSim/docs/ToolsAndLibraries.md).

On Linux, dependencies (OpenCV 5, Eigen3, spdlog, nlohmann_json) and tools (CMake, Ninja, GCC) are available directly on the system path:

```bash
# Configure
cmake -B build -S .

# Build
cmake --build build
```

### Running Tests

Tests live in `tests/` and build into a separate `GolfSimTests` runner (never into `GolfSim`). Every case is registered with CTest individually.

```bash
cmake --build build
ctest --test-dir build --output-on-failure      # all cases + the separation guards
ctest --test-dir build -R Kinematics            # one case by name
./build/GolfSimTests --list                     # case names
./build/GolfSimTests --filter Recorder          # substring match
./build/GolfSimTests --verbose Units            # show info-level log output from the code under test
```

Windows uses a multi-config generator, so pass the configuration to ctest:

```powershell
& $cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

All test filesystem output goes under `build/test_sandbox/`; `build/replays/` and `build/shot_history.json` are never touched by tests. Set `-DGOLFSIM_BUILD_TESTS=OFF` to skip the test target entirely.

### Coverage

Coverage uses Clang + `llvm-cov` (line, region/statement, branch and MC/DC) in a separate Debug build tree. `tools/coverage/thresholds.json` holds the ratchet: raise it whenever tests are added, never lower it.

```bash
cmake -B build-cov -S . -G Ninja -DCMAKE_BUILD_TYPE=Debug -DGOLFSIM_COVERAGE=ON \
      -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
cmake --build build-cov --target coverage       # runs ctest instrumented, writes the report, checks thresholds
xdg-open build-cov/coverage-html/index.html
ctest --test-dir build-cov -R CoverageGate      # re-check the last report against thresholds.json
```

GCC is also accepted by `GOLFSIM_COVERAGE` (gcov + `-fcondition-coverage`) as a cross-check; report it with `gcovr --exclude-throw-branches` (`sudo pacman -S gcovr`). Exclusion markers (`// LCOV_EXCL_LINE`, `LCOV_EXCL_START/STOP`, each with a reason) are honoured by the gate for lines and branches and are capped at 2 % of gated lines.

### Linux Device Permissions (one-time)

| Device | Group | Notes |
| :--- | :--- | :--- |
| `/dev/video*` (cameras) | `video` | Already accessible to the locally logged-in user via systemd-logind `uaccess`. |
| `/dev/ttyACM0` (strobe controller) | `uucp` | **Not** covered by `uaccess`. Run `sudo usermod -aG uucp $USER` and log out/in. |

### Linux Camera Selection

`uvcvideo` creates two `/dev/video*` nodes per camera (capture + metadata). `--left-cam N` / `--right-cam N` are **logical** indices over capture-capable nodes only, so the two cameras are `0` and `1` regardless of the raw node numbers. To pin a specific node:

```bash
./build/GolfSim --live --left-dev /dev/video2 --right-dev /dev/video0
# Stable across re-plugs:
./build/GolfSim --left-dev /dev/v4l/by-id/usb-Arducam_..._-video-index0
```

Cross-check what the driver negotiated:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/videoN --list-formats-ext     # GREY / NV12 / YUYV at 1280x800?
v4l2-ctl -d /dev/videoN -L                     # control ranges (exposure_time_absolute is in 100 us units)
v4l2-ctl -d /dev/videoN -C exposure_time_absolute
```

If the second camera fails `VIDIOC_STREAMON` with `ENOSPC`, USB bandwidth is exhausted — move it to a different USB controller / USB 3 port, or `sudo modprobe -r uvcvideo && sudo modprobe uvcvideo quirks=0x80`.
