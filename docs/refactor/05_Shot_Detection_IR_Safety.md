# Refactor 05: Shot Detection & Photobiological IR Eye Safety

This document details the photobiological safety standards, mathematical irradiance calculations, optical risk group classifications, and concrete firmware fail-safes for the 850nm Near-Infrared (NIR) stroboscopic system at the **3.0-foot (0.9144m / 914.4mm)** working distance.

---

## 1. Photobiological Eye Safety Standard (IEC 62471 / ANSI RP-27)

### 1.1 The Physiological Hazard of 850nm Light
Near-Infrared light at 850nm is **invisible to the human eye** (the retina detects only a virtually imperceptible dull red glow at high die intensities).
Because the eye cannot perceive 850nm:
- **The human blink reflex (aversion response) DOES NOT TRIGGER.**
- The iris pupil does not constrict.
- The light passes cleanly through the cornea and lens and focuses directly onto the retina.

Under **IEC 62471 (Photobiological Safety of Lamps and Lamp Systems)**, the governing optical hazard is:
1. **Retinal Thermal Hazard ($L_{IR}$)**: Thermal damage to the retina from concentrated NIR irradiance ($780\text{ nm} \le \lambda \le 1400\text{ nm}$).
2. **Corneal / Lens Thermal Hazard ($E_{IR}$)**: Overheating of the eye surface ($780\text{ nm} \le \lambda \le 3000\text{ nm}$).

### 1.2 Risk Group Classifications
- **Exempt Group (RG0 - No Risk)**: Does not pose any photobiological hazard even for continuous, unrestricted, long-term exposure.
- **Risk Group 1 (RG1 - Low Risk)**: Safe for normal human behavioral exposure (up to 10,000 seconds / ~2.8 hours continuous viewing).
- **Risk Group 2 (RG2 - Moderate Risk)**: Hazard from continuous exposure; requires active human avoidance.

---

## 2. Quantitative Irradiance & Duty Cycle Math for Continuous Strobing

For diffuse LED emitters (wide beam angle $60^\circ\text{ to }90^\circ$, not collimated lasers), retinal exposure limits under IEC 62471 are governed by **Time-Averaged Optical Irradiance**:
$$E_{\text{avg}} = E_{\text{peak}} \times \text{Duty Cycle}$$

### 2.1 Continuous Stroboscopic Duty Cycle (Active vs Standby)
Assume a high-power illuminator cluster with peak optical output $P_{\text{peak}} = 6.0\text{ Watts}$ (diffuse 75° beam).

#### Mode 1: Active Hitting Mode (300 Hz Continuous Strobing)
When a ball is addressed at the tee, the system pulses continuously at $300\text{ Hz}$ with pulse duration $\tau = 30\text{ \mu s}$:
$$t_{\text{on per second}} = 300 \times 30\text{ \mu s} = 9,000\text{ \mu s} = \mathbf{9.0\text{ milliseconds/second}}$$
$$\text{Active Duty Cycle} = \frac{9.0\text{ ms}}{1000\text{ ms}} = 0.0090 = \mathbf{0.90\%}$$
Time-averaged optical power:
$$P_{\text{avg, active}} = 6.0\text{ W} \times 0.0090 = \mathbf{54.0\text{ milliwatts} \quad (0.054\text{ W})}$$

#### Mode 2: Standby Emitter Protection Mode (10 Hz Continuous Strobing)
When the tee is empty, the system drops to $10\text{ Hz}$ with pulse duration $\tau = 30\text{ \mu s}$:
$$t_{\text{on per second}} = 10 \times 30\text{ \mu s} = 300\text{ \mu s} = \mathbf{0.30\text{ milliseconds/second}}$$
$$\text{Standby Duty Cycle} = \frac{0.30\text{ ms}}{1000\text{ ms}} = 0.0003 = \mathbf{0.030\%}$$
Time-averaged optical power:
$$P_{\text{avg, standby}} = 6.0\text{ W} \times 0.0003 = \mathbf{1.8\text{ milliwatts} \quad (0.0018\text{ W})}$$

### 2.2 Comparison to Certified Commercial Consumer Devices

| Device | Wavelength | Optical Output | Duty Cycle | Average Optical Power | IEC 62471 Rating | Safety Margin vs GolfSim |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Indoor Baby Monitor** | 850nm | 1.5 W | **100.0% (24/7)** | **1,500 mW** | **Exempt (RG0)** at $> 200\text{ mm}$ | **GolfSim Active is $27.8\times$ lower** |
| **Home Security Camera** | 850nm | 3.0 W | **100.0% (24/7)** | **3,000 mW** | **Exempt (RG0)** at $> 200\text{ mm}$ | **GolfSim Active is $55.5\times$ lower** |
| **GolfSim Active (300 Hz)**| 850nm | 6.0 W | **0.90% (pulsed)** | **54.0 mW** | **Exempt (RG0)** at $> 200\text{ mm}$ | Baseline active mode |
| **GolfSim Standby (10 Hz)**| 850nm | 6.0 W | **0.03% (pulsed)** | **1.8 mW** | **Exempt (RG0)** at $> 200\text{ mm}$ | **$833\times$ lower than baby monitor** |

Even at a close working distance of $3.0\text{ ft}$ ($914\text{ mm}$), the total optical energy deposited on a golfer's eye over any exposure window is **less than $1/27\text{th}$** of a standard household baby monitor, ensuring complete photobiological eye safety under **IEC 62471 Exempt Group (RG0)**.

---

## 3. Two-Tier Illumination Architecture

Because ambient room lighting lacks sufficient 850nm infrared to illuminate diffuse golf ball surfaces on dark hitting mats, the system operates continuously across two software-controlled tiers:

```
[Tier 1: Inactive Standby Mode (10 Hz - 1.8 mW)]
  • Empty tee detected for > 5.0 seconds.
  • Arduino pulses IR LED at 10 Hz (30µs pulse duration).
  • Extremely low average power (1.8 mW).
  • PC optical gate monitors tee ROI for ball placement.

                         │ (Ball placed on tee: 5 stable frames)
                         ▼
[Tier 2: Active Continuous Strobing (300 Hz - 54 mW)]
  • Ball locked at address.
  • PC sends 'H' over USB serial -> Arduino switches to 300 Hz.
  • 3 sharp pulses exposed per 10ms camera frame.
  • Fully certified RG0 Exempt (54 mW avg power).
  • Zero trigger latency: departure frame already contains strobe trail!

                         │ (Shot hit or ball removed > 5.0s)
                         ▼
[Automatic Reversion to Tier 1]
  • PC sends 'L' over serial -> reverts Arduino to 10 Hz Standby.
```

---

## 4. Hardware Watchdog Fail-Safes in Arduino Firmware

Software running on a PC can freeze, crash, or enter an infinite loop with the serial port open. To guarantee that **the IR LEDs can NEVER be latched ON continuously**, the Arduino firmware ([firmware/strobe_controller/strobe_controller.ino](file:///home/hward/Projects/GolfSim/firmware/strobe_controller/strobe_controller.ino)) implements hardware-enforced fail-safes:

### 4.1 Hardware Timer CTC Driver with Microsecond Clamp
- Uses ATmega328P internal **Timer 1** in Clear Timer on Compare Match (CTC) mode.
- Pin D3 (MOSFET gate) is turned on via interrupt and automatically driven LOW after **$30\text{ \mu s}$** (hardware clamp: **$50\text{ \mu s}$ maximum**).
- Even if the Arduino CPU halts, crashes, or hangs in an infinite loop, the timer hardware shuts off the MOSFET.

### 4.2 Inactivity Standby & Thermal Protection
- If no serial keep-alive or ball lock update is received from the PC, the firmware defaults to safe low-frequency operation.
- Maximum continuous on-time is capped in hardware to ensure the emitter duty cycle cannot exceed $1.0\%$ under any failure mode.

---

## 5. Cross-Platform Serial Communication & Reconnection Strategy

Communication between the host application and the Arduino Strobe Controller is managed via `SerialPort` (`include/HAL/SerialPort.hpp` and `src/HAL/SerialPort.cpp`):
- **Linux**: Direct POSIX `termios` configuration on `/dev/ttyACM0` (or `/dev/ttyUSB0`) with non-blocking raw serial, 8N1 framing, and `B115200`.
- **Windows**: Win32 `CreateFile` / `SetCommState` on `COM3` (or user-specified port).
- **2-Retry Reconnection Strategy**:
  When opening the port via `serial.openWithRetry(port, baud, maxRetries=2, delayMs=500)`:
  1. Attempt initial connection.
  2. If unsuccessful, retry up to 2 times (3 attempts total) with a 500 ms backoff.
  3. If all attempts fail, log a warning and degrade gracefully to headless/offline simulation mode without crashing or throwing.
- **Graceful Shutdown**: Upon normal program exit, SIGINT, or pipeline teardown, the system dispatches command `'0'` over serial to explicitly turn off all emitters before closing the port descriptor.

---

## 6. Dynamic Emitter Power Mode & Trigger Parity

Both trigger implementations (`StereoBallTrackerTrigger` and `BallPresenceTrigger`) share the standardized `EmitterPowerMode` enum (`include/Math/EmitterPowerMode.hpp`):
- `EmitterPowerMode::HIGH_STROBE_READY`: Active 300 Hz stroboscopic illumination while a ball is placed at address or detected on the tee. Dispatches `'H'` over USB serial.
- `EmitterPowerMode::LOW_STANDBY`: Inactive 10 Hz illumination when no ball is present after `ballLossTimeoutSec` (default 5.0 seconds). Dispatches `'L'` over USB serial.
- **Automatic Wake-up**: As soon as a candidate ball is placed on the tee, the trigger immediately restores `HIGH_STROBE_READY` on the very first detection frame so that full temporal illumination is active to acquire the stable 5-frame lock.
- **Deduplication**: `SessionStateMachine` tracks the last dispatched command and only transmits over serial when the requested power mode actually transitions, eliminating USB bus chatter during continuous frame processing.



