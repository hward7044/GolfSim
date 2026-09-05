# Refactor 03: Hardware Trigger Handshake & Latency Analysis

This document addresses the camera-to-microcontroller physical handshake, breaks down the critical timing limitations of USB communication on Windows and Linux, and provides concrete low-latency solutions.

---

## 1. Physical Hardware Handshake (Camera STROBE $\leftrightarrow$ Arduino)

### 1.1 Existing Hardware Wiring
- **Camera Module**: Dual Arducam OV9281. The OV9281 sensor has a dedicated hardware **STROBE** output pin.
- **Microcontroller**: Arduino Nano (ATmega328P).
- **Physical Connections**:
  - `Camera STROBE Pin` $\longrightarrow$ `Arduino Pin D2 (External Interrupt 0)`
  - `Arduino Pin D3` $\longrightarrow$ `Logic-Level N-Channel MOSFET Gate (IR LED Bank)`
  - `Common Ground` $\longrightarrow$ Shared between Camera, Arduino, and 12V LED Power Supply.

### 1.2 The Software-Armed Handshake Protocol
To prevent the Arduino from firing high-power strobes on every single preview/idle frame, the Arduino acts as an **armed gate**:

```
[Normal Idle State]
  • Arduino listens on Serial (115200 baud).
  • Internal flag: volatile bool strobeArmed = false.
  • Camera exposes idle frames at 30-60 FPS -> STROBE pin pulses D2 -> ISR immediately exits!

[Launch Detection: PC trips Optical Gate]
  • PC sends single byte 'F' over USB Serial to Arduino.
  • Arduino reads 'F' -> sets strobeArmed = true.

[Hardware-Locked Strobe Pulse]
  • On the VERY NEXT rising edge of Camera STROBE pin (D2):
    - ISR checks strobeArmed == true -> FIRES 5-PULSE STROBE TRAIN!
    - Sub-pulse 1: 30µs HIGH -> 970µs LOW
    - Sub-pulse 2: 30µs HIGH -> 970µs LOW
    - Sub-pulse 3: 30µs HIGH -> 970µs LOW
    - Sub-pulse 4: 30µs HIGH -> 970µs LOW
    - Sub-pulse 5: 30µs HIGH -> LOW
    - strobeArmed = false (automatically disarms!).
```

---

## 2. Deep Dive: USB Latency Analysis

> **Crucial Engineering Realization**: *"Windows and Linux USB drivers are not designed for sub-millisecond real-time hardware triggering."*

Let's calculate the exact end-to-end timing loop of a software-commanded USB trigger:

```
T0: Club strikes ball on the tee
 ├── T1 (+10.0 ms): Camera completes exposure of the hit frame.
 ├── T2 (+ 4.0 ms): UVC video driver reads frame over USB 2.0 and transfers to OpenCV.
 ├── T3 (+ 2.0 ms): PC consumer thread runs Optical Gate / template diff and detects departure.
 ├── T4 (+ 3.0 to 8.0 ms): Windows/Linux OS schedules USB CDC-ACM write & flushes serial buffer.
 └── T5: Arduino finally receives byte 'F' over Serial.
─────────────────────────────────────────────────────────────────────────────
Total Elapsed Latency from Impact to Arduino: 19.0 to 24.0 milliseconds!
```

### The Physical Impact at 2.0 ft Range:
At 80 mph ($35.8\text{ m/s}$), in $20\text{ ms}$ the ball travels:
$$\Delta x = 35.8\text{ m/s} \times 0.020\text{ s} = \mathbf{0.716\text{ meters} \quad (28\text{ inches})}$$
**The ball has already left the 2.0 ft field of view before the Arduino receives the `'F'` command!**

This is why reactive triggering (waiting until the ball moves in a video frame to send a serial command to turn on the strobe) cannot catch the departure frame using standard USB webcam drivers.

---

## 3. Low-Latency Solutions & Strategies

To solve this physical timing bottleneck, we evaluate four proven architectural strategies:

### Strategy A: Club-Entry / Pre-Impact Optical Trigger (Computer Vision)
- **Concept**: Instead of detecting the *ball leaving* the tee, the camera's optical gate monitors an ROI **2 to 4 inches behind the tee** (the clubhead entry zone).
- **Timing**: At 70 mph club speed, the club enters the frame **5 to 10 ms before impact**.
- **Execution**: Detecting the incoming club trips the trigger *before* contact. The PC sends `'F'` to the Arduino *in advance*, arming the strobe so it pulses during the exact frame of impact and initial launch.

### Strategy B: Continuous Low-Power Stroboscopic Operation
- **Concept**: If the IR strobe is running continuously at low duty-cycle (safe RG0 Exempt levels, see [05_Shot_Detection_IR_Safety.md](05_Shot_Detection_IR_Safety.md)), the camera is *already capturing multi-pulse images on every frame*.
- **Execution**: When a shot occurs, the ball trail is already recorded in the camera ring buffer. The PC simply pulls the frame that just occurred out of `AtomicRingBuffer` and solves it. Zero trigger latency required.

### Strategy C: Direct Hardware Sensor Trigger (The Ultimate Industry Standard)
- **Concept**: Connect a microsecond hardware sensor directly to an Arduino interrupt pin, completely bypassing the PC and USB stack:
  1. **Acoustic / Piezo Impact Sensor**: A small $2 piezo disc or contact microphone placed under the hitting mat near the tee, wired to Arduino Pin A0 / D3. When the club strikes the ball, the acoustic shockwave triggers the Arduino in $< 100\text{ microseconds}$!
  2. **Optical / Laser Beam Break**: A tiny $5 IR photogate 2 inches ahead of the ball.
- **Execution**: The Arduino detects the physical impact locally and pulses the IR strobe instantly in hardware without any USB latency. The PC's only job is reading the resulting frames.

### Strategy D: USB Serial Driver Latency Optimization (Software Tuning)
For whatever serial messages are sent over USB:
- **Windows**: In Device Manager $\to$ Advanced Port Settings for COM port, change **Latency Timer** from default $16\text{ ms}$ to **$1\text{ ms}$**.
- **Win32Serial C++ Implementation**: Configure `COMMTIMEOUTS` with non-blocking write completion and use `PurgeComm(hSerial, PURGE_TXCLEAR)` to prevent OS buffer stacking.
- **Linux**: Configure `termios` with `VMIN = 0, VTIME = 0` and use `ioctl(fd, TCFLSH, 2)`.

