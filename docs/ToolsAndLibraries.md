# Installed Tools and Libraries

This document details the core libraries, build tools, compilers, and hardware subsystems installed and configured in the **GolfSim** development environment.

---

## 1. Operating System & Environment

| Component | Detail |
| :--- | :--- |
| **Operating System** | Omarchy Linux (Arch Linux rolling release, `x86_64`) |
| **Kernel** | Linux `7.1.9-arch1-2` |
| **Package Managers** | `pacman` (system packages), `mise` (CLI runtime & tool manager), `~/.local` (user-installed headers/CMake packages) |

---

## 2. Core C++ Libraries & Dependencies

These are the primary third-party libraries consumed by `CMakeLists.txt` and linked into the `GolfSim` target:

### OpenCV
- **Installed Version:** `5.0.0` (Package: `opencv 5.0.0-9`)
- **CMake Config Directory:** `/usr/lib/cmake/opencv5`
- **Include Directory:** `/usr/include/opencv5`
- **Pkg-Config Module:** `opencv5`
- **Architectural Note:** Unlike OpenCV 4 (which bundled calibration and stereoscopy in `calib3d`), OpenCV 5 splits and restructures modules. `CMakeLists.txt` explicitly requires OpenCV 5.0 with the following components:
  - `core`
  - `imgproc`
  - `geometry` (OpenCV 5 specific)
  - `stereo` (OpenCV 5 specific)
  - `calib` (OpenCV 5 specific)
  - `imgcodecs`
  - `highgui`
  - `videoio`

### Eigen3
- **Installed Version:** `5.0.1` (`5.0.1-dev+bc3b3987`, SemVer transition where `EIGEN_WORLD_VERSION = 3` and `EIGEN_MAJOR_VERSION = 5`)
- **CMake Config Directory:** `/home/hward/.local/share/eigen3/cmake`
- **Include Directory:** `/home/hward/.local/include/eigen3`
- **CMake Target:** `Eigen3::Eigen`
- **Usage:** Matrix mathematics, 3D coordinate transformations, trajectory simulation, and ballistics modeling in `EigenBallisticsEngine` and `StereoTriangulator`.

### spdlog
- **Installed Version:** `1.17.0` (Package: `spdlog 1.17.0-2`)
- **CMake Config Directory:** `/usr/lib/cmake/spdlog`
- **Include Directory:** `/usr/include/spdlog`
- **CMake Target:** `spdlog::spdlog`
- **Usage:** High-performance structured logging subsystem utilized across all camera, trigger, and diagnostic nodes (`GlobalLogger`).

### nlohmann_json
- **Installed Version:** `3.12.0`
- **CMake Config Directory:** `/home/hward/.local/share/cmake/nlohmann_json`
- **Include Directory:** `/home/hward/.local/include/nlohmann`
- **CMake Target:** `nlohmann_json::nlohmann_json`
- **Usage:** Serialization and network transmission of launch monitor shot packets via TCP socket (`TcpJsonTransmitter`).

---

## 3. Compilers & Toolchains

| Tool | Version | Path / Source | Purpose |
| :--- | :--- | :--- | :--- |
| **GCC / G++** | `16.2.1 20260810` | `/usr/bin/g++`, `/usr/bin/gcc` | Primary C++ host compiler |
| **Clang / LLVM** | `22.1.8` | `/usr/bin/clang++`, `/usr/bin/clang` | Alternative compiler / tooling |
| **C++ Standard** | `C++20` | Configured via `CMAKE_CXX_STANDARD 20` | Core language standard |
| **ARM Embedded Toolchain** | `13.3.Rel1~amd64` | `gcc-arm-none-eabi` (via `mise`) | Firmware compilation for microcontroller / strobe controller |

---

## 4. Build Systems & Developer Utilities

| Tool | Version | Manager / Source | Notes |
| :--- | :--- | :--- | :--- |
| **CMake** | `4.4.3` | `mise` (`~/.config/mise/config.toml`) | Project configuration & build generation |
| **Ninja** | `1.13.2` | `mise` | High-speed build backend |
| **GNU Make** | `4.4.1` | `/usr/bin/make` | Alternative generator backend |
| **GDB** | `17.2` | `/usr/bin/gdb` | Native C++ debugging |
| **Git** | `2.55.0` | `/usr/bin/git` | Version control |

---

## 5. Hardware, Video & Serial Drivers

| Subsystem / Utility | Version | Implementation Details |
| :--- | :--- | :--- |
| **V4L2 (`v4l-utils`)** | `1.32.0` (`v4l2-ctl`) | Provides Linux video subsystem management for dual OV9281 mono global-shutter USB cameras (`src/HAL/V4L2Driver.cpp`). Handles direct `ioctl` camera controls (exposure, gain, pixel format). |
| **Serial Communication** | Native POSIX termios | Hardware strobe controller handshake over USB serial `/dev/ttyACM*` or `/dev/ttyUSB*` (`src/HAL/SerialPort.cpp`). |
| **Firmware Subsystem** | Arduino / C++ | Firmware source located in `firmware/strobe_controller/` targeting hardware microcontrollers. |

---

## 6. Scripting & Runtimes

| Runtime | Version | Notes |
| :--- | :--- | :--- |
| **Python** | `3.14.7` | Base system interpreter. Scripts in `tools/` (such as `analyze_replay_stream.py` and `debug_ir_strobe.py`) interface with recorded frame buffers and IR dot analysis. |
| **Node.js** | `26.8.1` | Managed via `mise`. |

---

## 7. Version Verification Commands

To verify installed versions on this system at any time, run:

```bash
# Compilers & Build Tools
cmake --version
ninja --version
g++ --version
make --version

# OpenCV 5
pkg-config --modversion opencv5
opencv_version

# V4L2 Utilities
v4l2-ctl --version

# mise tools
mise list
```

