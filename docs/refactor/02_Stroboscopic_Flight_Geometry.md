# Refactor 02: Stroboscopic Flight Geometry & Timing (3.0 ft Range)

This document details the optical geometry, kinematic timing models, sensor hardware constraints, and pulse parameter calculations for the **3.0-foot (0.9144m / 914.4 mm) working distance** launch monitor setup, focused on irons, wedges, woods, and putts with a scalable 1-to-2 frame hybrid architecture.

---

## 1. Physical Setup & Optical Constraints

### 1.1 Geometry at 3.0 Feet (914.4 mm)
- **Sensor**: Dual OmniVision OV9281 (1280 $\times$ 800, monochrome global shutter, $3.0\ \mu\text{m}$ OmniPixel3-GS pixel pitch).
- **USB Bridge**: Sonix Technology USB 2.0 Bridge (`0x0C45:0x6366`).
- **Lens**: 2.8mm focal length M12 low-distortion lens.
- **Stereo Baseline**: $B = 100\text{ mm}$ rigid rail.
- **Working Distance**: $Z = 3.0\text{ ft} = 0.9144\text{ m} = 914.4\text{ mm}$ (parallel from camera lenses to the tee flight line).

### 1.2 Field of View (FOV) at 3.0 ft
The sensor active area is:
- Width: $W_{\text{sensor}} = 1280 \times 0.003\text{ mm} = 3.84\text{ mm}$
- Height: $H_{\text{sensor}} = 800 \times 0.003\text{ mm} = 2.40\text{ mm}$

Horizontal and Vertical FOV angles:
$$\theta_H = 2 \arctan\left(\frac{3.84}{2 \times 2.8}\right) = 2 \arctan(0.6857) \approx 68.8^\circ$$
$$\theta_V = 2 \arctan\left(\frac{2.40}{2 \times 2.8}\right) = 2 \arctan(0.4286) \approx 46.4^\circ$$

At working distance $Z = 0.9144\text{ m}$ (3.0 ft):
$$\text{Horizontal FOV Window } W_{\text{FOV}} = 2 \times Z \times \tan(34.4^\circ) \approx 2 \times 0.9144 \times 0.6857 \approx \mathbf{1.254\text{ m} \quad (\approx 49.4\text{ inches} \approx 4.1\text{ ft})}$$
$$\text{Vertical FOV Window } H_{\text{FOV}} = 2 \times Z \times \tan(23.2^\circ) \approx 2 \times 0.9144 \times 0.4286 \approx \mathbf{0.784\text{ m} \quad (\approx 30.9\text{ inches})}$$

Accounting for edge margins and stereo overlap between Left and Right cameras, the **effective visible flight baseline** is:
$$L_{\text{flight}} \approx 0.85\text{ m to } 1.05\text{ m} \quad (\approx 33\text{ to } 41\text{ inches})$$

### 1.3 Apparent Ball Size on Sensor
A standard regulation golf ball has diameter $D_{\text{ball}} = 42.67\text{ mm}$ ($r = 21.335\text{ mm}$).
Effective focal length in pixels:
$$f_x \approx \frac{2.8\text{ mm}}{0.003\text{ mm/pixel}} \approx 933\text{ pixels (nominal } 1000\text{ px calibrated)}$$
Apparent ball diameter on sensor:
$$D_{\text{px}} \approx 1000 \times \frac{0.04267\text{ m}}{0.9144\text{ m}} \approx \mathbf{46.7\text{ pixels}} \quad (\text{Radius } R_{\text{px}} \approx \mathbf{23.3\text{ px}})$$
- Pixel area: $\text{Area} \approx \pi \times (23.3)^2 \approx 1,700\text{ px}^2$.
- At 3.0 ft, this provides cleaner spatial separation between successive strobe pulses while remaining well above the tracker's minimum detection threshold ($80\text{ px}^2$).

---

## 2. Sensor Hardware Timing Guide (Arducam OV9281 USB)

### 2.1 Hardware Capabilities & Limitations
1. **USB Bridge (Sonix 0c45:6366)**:
   - Does **not** support raw I2C sensor register manipulation over UVC Extension Units.
   - Does **not** break out the OV9281 sensor's `FSIN` (hardware external trigger) pin.
2. **Operational Architecture**:
   - Camera runs in **continuous 100 FPS streaming mode** ($T_{\text{frame}} = 10.0\text{ ms}$).
   - Exposure is set to $10.0\text{ ms}$ ($10,000\ \mu\text{s}$) via standard UVC driver calls (`IAMCameraControl` / `v4l2_control`).
   - Strobe pulsing is driven independently by the **Arduino microcontroller (ATmega328P)** switching a logic-level MOSFET connected to the 850nm IR LED bank.

### 2.2 Kinematic Timing Guide Across Shot Speeds (at 3.0 ft / 90 cm FOV)

| Club / Shot Class | Ball Speed (mph) | Ball Speed (m/s) | Travel per 1.0ms Pulse | Travel in 10ms Frame | Time across 0.90m FOV | Frames in FOV | Solve & Capture Behavior |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Putt / Chip** | 15 mph | $6.7\text{ m/s}$ | $6.7\text{ mm}$ | $6.7\text{ cm}$ | **134 ms** | **13 frames** | Slow movement; solves after `maxFramesPerShot = 2` (10 pulses across 13.4 cm). |
| **Sand / Lob Wedge** | 50 mph | $22.4\text{ m/s}$ | $22.4\text{ mm}$ | $22.4\text{ cm}$ | **40.2 ms** | **4 frames** | Frame 1: 5 pulses (22.4 cm). Frame 2: 5 pulses (44.8 cm). Solves after Frame 2 across 10 pulses. |
| **PW / 9-Iron** | 80 mph | $35.8\text{ m/s}$ | $35.8\text{ mm}$ | $35.8\text{ cm}$ | **25.1 ms** | **2–3 frames** | Frame 1: 5 pulses (35.8 cm). Frame 2: 5 pulses (71.6 cm). Solves immediately after Frame 2 across 72 cm baseline. |
| **6-Iron / 7-Iron** | 105 mph | $46.9\text{ m/s}$ | $46.9\text{ mm}$ | $46.9\text{ cm}$ | **19.2 ms** | **2 frames** | Complete pulse separation ($46.9\text{ mm} > 42.7\text{ mm}$ ball dia). Solves after Frame 2. |
| **3-Wood / Driver** | 160 mph | $71.5\text{ m/s}$ | $71.5\text{ mm}$ | $71.5\text{ cm}$ | **12.6 ms** | **1–2 frames** | Ball travels 71.5 cm in Frame 1 and exits during Frame 2. Solves immediately on Frame 2 empty frame (<15 ms latency). |

---

## 3. Pulse Spacing & Aliasing Math

To determine the strobe pulse interval $\Delta t$, we balance two physical constraints:
1. **Rotational Aliasing Boundary**: Rotation between pulses must be $< 90^\circ$ (ideally $< 45^\circ\text{–}60^\circ$) so Procrustes SVD can uniquely track retroreflective marker dots without ambiguous $360^\circ$ wrap-around.
2. **Blob Spatial Separation**: Consecutive ball silhouettes should ideally not overlap too heavily ($> 50\%$).

### 3.1 Spin Aliasing Constraint
For high-spin wedge shots: $\text{Spin} \approx 9,000\text{ RPM} = 150\text{ rev/s} = 300\pi\text{ rad/s} \approx 942.5\text{ rad/s}$.
Angular rotation $\Delta \theta = \omega \cdot \Delta t$:
- At $\Delta t = 1.0\text{ ms}$: $\Delta \theta = 942.5 \times 0.0010 = 0.94\text{ rad} \approx \mathbf{54.0^\circ}$ (ideal margin, well under $90^\circ$).
- At $\Delta t = 0.8\text{ ms}$: $\Delta \theta = 942.5 \times 0.0008 = 0.75\text{ rad} \approx \mathbf{43.2^\circ}$.

### 3.2 Spatial Separation Constraint
At 80 mph ($35.8\text{ m/s}$):
- Travel per $\Delta t = 1.0\text{ ms}$: $\Delta x = 35.8 \times 0.001 = 0.0358\text{ m} = \mathbf{35.8\text{ mm}}$.
- Since ball diameter is $42.7\text{ mm}$, centers are $35.8\text{ mm}$ apart $\implies$ silhouettes slightly overlap by $6.9\text{ mm}$ (~16%).
- At 105 mph ($46.9\text{ m/s}$): $\Delta x = 46.9\text{ mm} > 42.7\text{ mm}$, completely clear separation!

### 3.3 Recommended Strobe Profile for 3.0 ft Setup
- **Number of Sub-pulses $N$**: 5 pulses per frame.
- **Pulse Interval $\Delta t$**: $1.0\text{ ms}$ (total pulse train duration = $4.0\text{ ms}$, easily fitting inside the 10.0 ms exposure window).
- **Sub-pulse Flash Duration**: $30\text{ \mu s}$ (freezes motion blur at 100 mph to under $1.4\text{ mm}$ / 1.5 pixels).

---

## 4. 1-to-2 Frame Hybrid Architecture

```
Shot Impact ──> [Frame 1 Capture (10ms exposure)] ──> Contains 5 pulses (0 to 36 cm)
                                                            │
                                  ┌─────────────────────────┴─────────────────────────┐
                                  ▼                                                   ▼
                    If ball exits FOV during Frame 1               If ball still in FOV (Irons / Wedges)
                    (High Speed Shot):                             (Moderate / Slow Shot):
                    • Solve kinematics immediately                 • Capture Frame 2 (next 10ms)
                    • Transmit TCP payload in <15ms                • Append Frame 2 pulses (36 to 72 cm)
                                                                   • Solve across 10 pulses with 72cm baseline!
```

### 4.1 Eliminating the Artificial 15-Frame Latency Bug
In the existing `SessionStateMachine.hpp`, the state machine had:
```cpp
if (emptyFrameCount >= 15 || trajectoryBuffer.size() >= 25)
```
Waiting 15 empty frames at 30–100 FPS forced the system to wait **150 to 500 ms** before computing and transmitting shot data.

### 4.2 Central Configuration Structure (`PipelineTimingConfig`)

```cpp
struct PipelineTimingConfig {
    double workingDistanceMeters = 0.9144; // 3.0 ft (914.4 mm)
    double pulseIntervalMs       = 1.0;    // 1.0 ms strobe spacing
    int    minPointsToSolve      = 3;      // Minimum pulses required to solve (default: 3)
    int    maxFramesPerShot      = 2;      // Cap at 2 frames maximum for irons/wedges
    int    emptyFrameTimeout     = 1;      // Solve immediately on 1st empty frame after pulses
    double subPulseDurationUs    = 30.0;   // 30 us LED strobe flash duration
    double nominalBallRadiusPx   = 23.3;   // ~23.3 px radius at 3.0 ft working distance
};
```
