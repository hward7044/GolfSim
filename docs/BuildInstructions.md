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
