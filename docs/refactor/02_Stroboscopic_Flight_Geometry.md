# Refactor 02: Stroboscopic Flight Geometry & Timing (2.0 ft Range)

This document details the optical geometry, kinematic timing models, and pulse parameter calculations for the **2.0-foot (0.61m) working distance** launch monitor setup, focused on irons and wedges with scalable multi-frame architecture.

---

## 1. Physical Setup & Optical Constraints

### 1.1 Geometry at 2.0 Feet (610 mm)
- **Sensor**: Dual OmniVision OV9281 (1280 $\times$ 800, monochrome global shutter, $3.0\text{ \mu m}$ pixel pitch).
- **Lens**: 2.8mm focal length M12 low-distortion lens.
- **Stereo Baseline**: $B = 100\text{ mm}$ rigid rail.
- **Working Distance**: $Z = 2.0\text{ ft} = 0.6096\text{ m}$ (parallel from camera lenses to the tee flight line).

### 1.2 Field of View (FOV) at 2.0 ft
The sensor active width is $W_{\text{sensor}} = 1280 \times 0.003\text{ mm} = 3.84\text{ mm}$.
Horizontal FOV angle $\theta_H$:
$$\theta_H = 2 \arctan\left(\frac{3.84}{2 \times 2.8}\right) = 2 \arctan(0.6857) \approx 68.8^\circ$$

At working distance $Z = 0.61\text{ m}$:
$$\text{Horizontal FOV Window } W_{\text{FOV}} = 2 \times Z \times \tan(34.4^\circ) \approx 2 \times 0.61 \times 0.6857 \approx \mathbf{0.836\text{ m} \quad (\approx 33\text{ inches})}$$
Accounting for edge margins and stereo overlap between Left and Right cameras, the **effective visible flight baseline** is:
$$L_{\text{flight}} \approx 0.50\text{ m to } 0.65\text{ m} \quad (\approx 20\text{ to } 26\text{ inches})$$

### 1.3 Apparent Ball Size on Sensor
A standard regulation golf ball has diameter $D_{\text{ball}} = 42.67\text{ mm}$ ($r = 21.335\text{ mm}$).
Effective focal length in pixels:
$$f_x \approx \frac{2.8\text{ mm}}{0.003\text{ mm/pixel}} \approx 933\text{ pixels (nominal } 1000\text{ px calibrated)}$$
Apparent ball diameter on sensor:
$$D_{\text{px}} \approx 1000 \times \frac{0.04267\text{ m}}{0.6096\text{ m}} \approx \mathbf{70\text{ pixels}} \quad (\text{Radius } R_{\text{px}} \approx 35\text{ px})$$
*(This confirms the `radius = 35 px` parameter in `MathTests.cpp` is physically exact for a 2-foot setup!)*

---

## 2. Kinematic Timing for Iron Shots

The initial hardware deployment targets **irons and wedges** (50 to 115 mph ball speed):

| Club Class | Ball Speed (mph) | Ball Speed (m/s) | Time across 0.55m FOV | Travel in 10ms Exposure |
| :--- | :--- | :--- | :--- | :--- |
| **Sand / Lob Wedge** | 55 mph | $24.6\text{ m/s}$ | **22.4 ms** | **24.6 cm** |
| **Pitching Wedge / 9-Iron** | 80 mph | $35.8\text{ m/s}$ | **15.4 ms** | **35.8 cm** |
| **6-Iron / 7-Iron** | 105 mph | $46.9\text{ m/s}$ | **11.7 ms** | **46.9 cm** |
| **3-Iron / Hybrid** | 125 mph | $55.9\text{ m/s}$ | **9.8 ms** | **55.9 cm** |

---

## 3. Pulse Spacing & Aliasing Math

To determine the strobe pulse interval $\Delta t$, we balance two physical constraints:
1. **Rotational Aliasing Boundary**: Rotation between pulses must be $< 90^\circ$ (ideally $< 45^\circ\text{–}60^\circ$) so Procrustes SVD can uniquely track retroreflective marker dots without ambiguous $360^\circ$ wrap-around.
2. **Blob Spatial Separation**: Consecutive ball silhouettes should ideally not overlap too heavily ($> 50\%$).

### 3.1 Spin Aliasing Constraint
For high-spin wedge shots: $\text{Spin} \approx 9,000\text{ RPM} = 150\text{ rev/s} = 300\pi\text{ rad/s} \approx 942.5\text{ rad/s}$.
Angular rotation $\Delta \theta = \omega \cdot \Delta t$:
- At $\Delta t = 1.2\text{ ms}$: $\Delta \theta = 942.5 \times 0.0012 = 1.13\text{ rad} \approx \mathbf{64.8^\circ}$ (well under $90^\circ$ aliasing limit).
- At $\Delta t = 0.8\text{ ms}$: $\Delta \theta = 942.5 \times 0.0008 = 0.75\text{ rad} \approx \mathbf{43.2^\circ}$ (ideal margin).

### 3.2 Spatial Separation Constraint
At 80 mph ($35.8\text{ m/s}$):
- Travel per $\Delta t = 1.0\text{ ms}$: $\Delta x = 35.8 \times 0.001 = 0.0358\text{ m} = \mathbf{35.8\text{ mm}}$.
- Since ball diameter is $42.7\text{ mm}$, centers are $35.8\text{ mm}$ apart $\implies$ silhouettes slightly overlap by $6.9\text{ mm}$ (~16%).
- OpenCV Moments tracker handles this smoothly, and the Hough circle fallback separates overlapping circles.

### 3.3 Recommended Strobe Profile for Irons Setup
- **Number of Sub-pulses $N$**: 5 pulses per frame.
- **Pulse Interval $\Delta t$**: $1.0\text{ ms}$ (total pulse train duration = $4.0\text{ ms}$, easily fitting inside the 10.0 ms exposure window).
- **Sub-pulse Flash Duration**: $30\text{ \mu s}$ (freezes motion blur at 100 mph to under $1.4\text{ mm}$ / 2.3 pixels).

---

## 4. 1-to-2 Frame Hybrid Architecture

```
Shot Impact ──> [Frame 1 Capture (10ms exposure)] ──> Contains 5 pulses (0 to 35 cm)
                                                            │
                                  ┌─────────────────────────┴─────────────────────────┐
                                  ▼                                                   ▼
                    If ball exits FOV during Frame 1               If ball still in FOV (Iron / Wedge)
                    (High Speed Shot):                             (Moderate / Slow Shot):
                    • Solve kinematics immediately                 • Capture Frame 2 (next 10ms)
                    • Transmit TCP payload in <15ms                • Append Frame 2 pulses (35 to 60 cm)
                                                                   • Solve across 10 pulses with 60cm baseline!
```

### 4.1 Eliminating the Artificial 15-Frame Latency Bug
In the existing `SessionStateMachine.hpp`, the state machine had:
```cpp
if (emptyFrameCount >= 15 || trajectoryBuffer.size() >= 25)
```
Waiting 15 empty frames at 30–100 FPS forced the system to wait **150 to 500 ms** before computing and transmitting shot data.

### 4.2 Configurable Threshold Bookmarks
We will expose these parameters as clear, tunable settings in a central configuration structure:

```cpp
struct PipelineTimingConfig {
    double workingDistanceMeters = 0.6096; // 2.0 ft
    double pulseIntervalMs       = 1.0;    // 1.0 ms strobe spacing
    int    minPointsToSolve      = 3;      // Minimum pulses required to solve (default: 3)
    int    maxFramesPerShot      = 2;      // Cap at 2 frames maximum for irons
    int    emptyFrameTimeout     = 1;      // Solve immediately on 1st empty frame after pulses
};
```
This gives the user complete freedom to tweak solve timing and club-specific tuning without recompilation.

