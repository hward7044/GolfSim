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
Under continuous $300\text{ Hz}$ strobing ($\Delta t = 3.333\text{ ms}$, 3 pulses per 10ms exposure):

| Club / Shot Class | Ball Speed (mph) | Ball Speed (m/s) | Travel per 3.33ms Pulse | Travel in 10ms Frame | Time across 0.90m FOV | Frames in FOV | Solve & Capture Behavior |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Putt / Chip** | 15 mph | $6.7\text{ m/s}$ | $2.2\text{ cm}$ | $6.7\text{ cm}$ | **134 ms** | **13 frames** | Slow movement; solves after `maxFramesPerShot = 2` (6 pulses across 13.4 cm). |
| **Sand / Lob Wedge** | 50 mph | $22.4\text{ m/s}$ | $7.5\text{ cm}$ | $22.4\text{ cm}$ | **40.2 ms** | **4 frames** | Frame 1: 3 pulses (22.4 cm). Frame 2: 3 pulses (44.8 cm). Solves after Frame 2 across 6 pulses. |
| **PW / 9-Iron** | 80 mph | $35.8\text{ m/s}$ | $11.9\text{ cm}$ | $35.8\text{ cm}$ | **25.1 ms** | **2–3 frames** | Frame 1: 3 pulses (35.8 cm). Frame 2: 3 pulses (71.6 cm). Completely clean silhouette separation ($11.9\text{ cm} > 4.3\text{ cm}$). |
| **6-Iron / 7-Iron** | 105 mph | $46.9\text{ m/s}$ | $15.6\text{ cm}$ | $46.9\text{ cm}$ | **19.2 ms** | **2 frames** | Complete pulse separation. Solves after Frame 2 across 70+ cm baseline. |
| **3-Wood / Driver** | 160 mph | $71.5\text{ m/s}$ | $23.8\text{ cm}$ | $71.5\text{ cm}$ | **12.6 ms** | **1–2 frames** | Ball travels 71.5 cm in Frame 1 (3 pulses) and exits during Frame 2. Solves immediately on Frame 2 empty frame (<15 ms latency). |

---

## 3. Pulse Spacing & Spatial Separation Math

To determine the strobe pulse interval $\Delta t$, we balance two physical constraints:
1. **Blob Spatial Separation**: Consecutive ball silhouettes should not overlap so moments contour analysis extracts crisp individual centroids.
2. **Exposure Budget**: $N$ sub-pulses must comfortably fit within the camera's $10.0\text{ ms}$ global shutter exposure.

### 3.1 Spatial Separation at 300 Hz ($\Delta t = 3.333\text{ ms}$)
At 80 mph ($35.8\text{ m/s}$):
- Travel per $\Delta t = 3.333\text{ ms}$: $\Delta x = 35.8 \times 0.00333 = \mathbf{119.3\text{ mm} \quad (\approx 11.9\text{ cm})}$.
- Since ball diameter is $42.67\text{ mm}$, centers are $119.3\text{ mm}$ apart $\implies$ **silhouettes are separated by $76.6\text{ mm}$ with zero overlap!**
- Even for chip shots down to 30 mph ($13.4\text{ m/s}$), travel is $44.7\text{ mm} > 42.7\text{ mm}$, maintaining clean silhouette separation without blob blending.

### 3.2 Final Production Strobe Profile (Implemented)
- **Active Frequency**: $300\text{ Hz}$ continuous strobing when ball is locked at address.
- **Pulse Interval $\Delta t$**: $3.3333\text{ ms}$ ($3,333\ \mu\text{s}$).
- **Pulses per Frame**: 3 pulses during each $10.0\text{ ms}$ exposure window.
- **Sub-pulse Flash Duration**: $30\text{ \mu s}$ (freezes motion blur at 100 mph to under $1.4\text{ mm}$ / 1.5 pixels).
- **Standby Frequency**: $10\text{ Hz}$ when tee is empty (drops optical power to $< 0.03\%$ duty cycle).

---

## 4. 1-to-2 Frame Hybrid Architecture

```
Continuous 300 Hz Strobing
          │
Impact Departure ──> [Frame 1 Capture (10ms exposure)] ──> Contains 3 pulses (0 to 36 cm)
                                                                 │
                                ┌────────────────────────────────┴────────────────────────────────┐
                                ▼                                                                 ▼
                  If ball exits FOV during Frame 1                              If ball still in FOV (Irons / Wedges)
                  (High Speed Shot):                                            (Moderate / Slow Shot):
                  • Solve kinematics immediately                                • Capture Frame 2 (next 10ms)
                  • Transmit TCP payload in <15ms                               • Append Frame 2 pulses (36 to 72 cm)
                                                                                • Solve across up to 6 pulses with 72cm baseline!
```

### 4.1 Eliminating the Artificial 15-Frame Latency Bug
In the legacy `SessionStateMachine.hpp`, the state machine waited for 15 empty frames before solving (`if (emptyFrameCount >= 15)`), adding an unacceptable 150–500 ms delay.
The refactored state machine uses `emptyFrameTimeout = 1` from `PipelineTimingConfig`, solving immediately upon the first empty frame following ball departure.

### 4.2 Central Configuration Structure (`PipelineTimingConfig`)
Implemented in [include/Orchestration/PipelineTimingConfig.hpp](file:///home/hward/Projects/GolfSim/include/Orchestration/PipelineTimingConfig.hpp):

```cpp
struct PipelineTimingConfig {
    double workingDistanceMeters = 0.9144; // 3.0 ft (914.4 mm)
    double pulseIntervalMs       = 3.3333; // 300 Hz continuous strobing spacing
    int    minPointsToSolve      = 3;      // Minimum pulses required to solve
    int    maxFramesPerShot      = 2;      // Cap at 2 frames maximum for irons/wedges
    int    emptyFrameTimeout     = 1;      // Solve immediately on 1st empty frame after pulses
    double subPulseDurationUs    = 30.0;   // 30 us LED strobe flash duration
    double nominalBallRadiusPx   = 23.3;   // ~23.3 px radius at 3.0 ft working distance
    double highStrobeRateHz      = 300.0;  // Continuous active rate
    double standbyStrobeRateHz   = 10.0;   // Inactivity standby protection rate
    int    cameraExposureUs      = 10000;  // 10.0 ms camera exposure window
    int    strobePulseCount      = 3;      // 3 pulses per 10ms exposure
    double ballLossTimeoutSec    = 5.0;    // 5.0s empty tee timeout to standby
};
```
