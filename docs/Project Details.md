================================================================================
PROJECT: DIY STEREOSCOPIC GOLF LAUNCH MONITOR (C++20)
TARGET: OPEN GOLF SIMULATOR INTEGRATION
================================================================================

1. CORE SYSTEM ARCHITECTURE & ENVIRONMENT
--------------------------------------------------------------------------------
- Language: C++20 (Strictly multi-threaded, lock-free architecture).
- Deployment: Windows (Development) / Linux Desktop via Valve Proton (Production).
- Compute: Single-PC Monolithic architecture. C++ tracks in background; GPU renders sim.
- Physical Footprint: 12-foot deep room. Cameras mounted on the floor, 3.5 to 4.0 feet parallel to the tee.
- Illumination: EYE-SAFE INFRARED STROBE. System relies on invisible 850nm IR illumination to prevent visual distraction/migraines. The lights run in low-power mode during standby and pulse at high-intensity only during active frame integration.

2. HARDWARE & SENSING CONFIGURATION
--------------------------------------------------------------------------------
- Sensor: Dual Arducam/OmniVision OV9281 (1MP Monochrome Global Shutter). No IR-cut filter.
- Lenses: 2.8mm focal length M12 low-distortion lenses yielding a ~56-64 inch horizontal FOV window.
- Baseline: Sensors mounted EXACTLY 100mm apart (center-to-center) on a rigid rail.
- Target Exposure: Global shutter clamped manually at <= 37.8 microseconds to keep motion blur under 0.1 inches at 150 mph ball speeds (protecting Sharpie dots for spin tracking).
- Strobe Interactivity: Physical STROBE pins on both OV9281 sensors are wired to an external N-Channel MOSFET switching circuit (e.g., IRF520) driven by a Mean Well 12V/24V DC power supply to fire the high-power 850nm IR LED arrays with zero OS latency.

3. IMPLEMENTATION DETAIL 1: THE HAL (PRODUCER THREAD)
--------------------------------------------------------------------------------
- Goal: Low-power idle seeking, transitioning to zero-latency raw burst frame capture upon launch.
- API: Do not use cv::VideoCapture. Use native OS APIs via preprocessor macros. Must implement out-of-band USB Extension Unit (XU) control transfers to inject immediate I2C register writes to the sensor.
- Linux (#ifdef __linux__): Use V4L2 C API with Memory-Mapped IO (mmap) to read straight from hardware RAM.
- Windows (#ifdef _WIN32): Use Microsoft Media Foundation (IMFMediaSource).
- Loop: High-priority std::jthread. While IDLE, it commands the camera to stream at low exposure. When a hit is confirmed, it injects immediate I2C writes to the registers to switch the sensor to snapshot strobe mode, capturing a rapid cache of highly illuminated frames. NO MATH ALLOWED HERE.

4. IMPLEMENTATION DETAIL 2: THE ATOMIC RING BUFFER
--------------------------------------------------------------------------------
- Goal: Lock-free memory bridge between Producer (Hardware) and Consumer (Math).
- Structure: std::array<FrameSet, SIZE> where SIZE is a power of 2 (e.g., 16 or 32).
- Thread Safety: Use std::atomic<size_t> head and tail indices for non-blocking read/write synchronization using memory order fences.
- Memory Rule: PRE-ALLOCATE all cv::Mat arrays at boot. Do not call new, malloc, or resize inside the capture loop. Serves as a high-precision burst cache to absorb 120+ FPS hardware frame delivery while the consumer thread processes the linear algebra.

5. IMPLEMENTATION DETAIL 3: OPTICAL GATE TRIGGER
--------------------------------------------------------------------------------
- Goal: Robust, environment-agnostic swing detection independent of sound or static tee placement.
- Setup: Define a fixed cv::Rect (Region of Interest) bounding box in the air 3 to 6 inches downrange from the physical tee location.
- Trigger Math: Run a lightweight cv::absdiff() exclusively on this small ROI box between Frame(N) and a static background reference frame to count non-zero pixel deltas.
- State Transition: When the ball crosses into the downrange box and pixel changes surpass MIN_BALL_PIXELS, the C++ producer instantly flags a hit, flashes the high-power IR strobe, and locks down the subsequent high-contrast flight frames.

6. IMPLEMENTATION DETAIL 4: 2D CENTROID TRACKING (OPENCV)
--------------------------------------------------------------------------------
- Goal: Sub-pixel 2D tracking without sluggish HoughCircles.
- Step 1: Binarize ROI using cv::threshold to turn the white ball into solid 255. High contrast from the matte black backdrop ensures clean masking.
- Step 2: Use cv::findContours() and filter by area to isolate the ball blob.
- Step 3: Pass contour to cv::moments().
- Formula: Extract center (x_c, y_c) using: x_c = (m10 / m00), y_c = (m01 / m00).

7. IMPLEMENTATION DETAIL 5: 3D TRIANGULATION
--------------------------------------------------------------------------------
- Goal: Convert 2D camera pixels into a Cartesian (X,Y,Z) trajectory array.
- Step 1: Load intrinsic/extrinsic matrices generated from cv::stereoCalibrate.
- Step 2: Apply cv::stereoRectify to align the left/right images on a horizontal 1D plane.
- Step 3: Calculate horizontal disparity (d).
- Formula: Solve for depth (Z) using: Z = (focal_length * baseline) / d. Apply via cv::triangulatePoints.

8. IMPLEMENTATION DETAIL 6: SPIN CALCULATION (EIGEN SVD)
--------------------------------------------------------------------------------
- Goal: Extract Spin RPM and Axis from marked balls.
- Step 1: Map 3D markings from Frame 1 (P1) and Frame 2 (P2) onto a normalized unit sphere.
- Step 2: Center the matrices and compute the cross-covariance matrix H.
- Step 3: Solve Orthogonal Procrustes using Eigen::JacobiSVD to decompose H into U, Sigma, V^T.
- Step 4: Compute Rotation Matrix (R = V * U^T).
- RPM Formula: Extract trace of R. Angle = arccos((Trace(R) - 1) / 2). RPM = (Angle / delta_time) * (60 / 2*pi).
- Axis: Extract the eigenvector of R corresponding to the eigenvalue of 1.

9. IMPLEMENTATION DETAIL 7: TCP TELEMETRY
--------------------------------------------------------------------------------
- Goal: Deliver payload to OpenGolfSim in < 15ms.
- Connection: Asynchronous TCP socket to 127.0.0.1 on Port 3111 (or 49152). Use Winsock2 (Windows) / sys/socket.h (Linux).
- Payload: Construct using nlohmann/json. 
- Keys Required: "type": "shot", "ballSpeed", "verticalLaunchAngle", "horizontalLaunchAngle", "spinSpeed", "spinAxis".
- Delivery: Convert to string (.dump()), append '\n', and send() over the stream.

10. DIAGNOSTICS
--------------------------------------------------------------------------------
- Global Logger: Use spdlog wrapped in a Singleton macro (LOG_INFO) for zero-overhead text debugging.
- Flight Recorder: Deep copy FrameSets during FLIGHT_CAPTURE and save them to disk ONLY AFTER the TCP payload is sent, allowing for offline replay/debugging without impacting launch latency.