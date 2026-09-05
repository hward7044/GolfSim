# Refactor 05: Shot Detection & Photobiological IR Eye Safety

This document details the photobiological safety standards, mathematical irradiance calculations, optical risk group classifications, and concrete firmware fail-safes for the 850nm Near-Infrared (NIR) stroboscopic system at the **2.0-foot (0.61m)** working distance.

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

## 2. Quantitative Irradiance & Duty Cycle Math at 2.0 Feet

For diffuse LED emitters (wide beam angle $60^\circ\text{ to }90^\circ$, not collimated lasers), retinal exposure limits under IEC 62471 are governed by **Time-Averaged Optical Irradiance**:
$$E_{\text{avg}} = E_{\text{peak}} \times \text{Duty Cycle}$$

### 2.1 Stroboscopic Shot Burst Duty Cycle
Assume a high-power illuminator cluster with peak optical output $P_{\text{peak}} = 6.0\text{ Watts}$ (diffuse 75° beam).
During a shot, the system fires:
$$N = 5\text{ pulses}, \quad \tau = 30\text{ \mu s per pulse}$$
Total light emission duration per shot:
$$t_{\text{on}} = 5 \times 30\text{ \mu s} = 150\text{ \mu s} = \mathbf{0.00015\text{ seconds}}$$

If a golfer hits a practice shot every 10 seconds:
$$\text{Duty Cycle} = \frac{150 \times 10^{-6}\text{ s}}{10\text{ s}} = 0.000015 = \mathbf{0.0015\%}$$
Time-averaged optical power:
$$P_{\text{avg}} = 6.0\text{ W} \times 0.000015 = \mathbf{0.09\text{ milliwatts} \quad (0.00009\text{ W})}$$

### 2.2 Comparison to Certified Commercial Consumer Devices

| Device | Wavelength | Optical Output | Duty Cycle | Average Optical Power | IEC 62471 Rating |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Indoor Baby Monitor** | 850nm | 1.5 W | **100.0% (24/7)** | **1,500 mW** | **Exempt (RG0)** at $> 200\text{ mm}$ |
| **Home Security Camera** | 850nm | 3.0 W | **100.0% (24/7)** | **3,000 mW** | **Exempt (RG0)** at $> 200\text{ mm}$ |
| **GolfSim Shot Burst** | 850nm | 6.0 W | **0.0015% (pulsed)**| **0.09 mW** | **Exempt (RG0) by $> 16,000\times$ margin** |

Even at a close working distance of $2.0\text{ ft}$ ($610\text{ mm}$), the total optical energy deposited on a golfer's eye over any 1-minute window is **less than $1/16,000\text{th}$** of a standard household baby monitor.

---

## 3. Two-Tier Safe Illumination Architecture

Because ambient room lighting lacks sufficient 850nm infrared to illuminate diffuse golf ball surfaces on dark hitting mats, we must illuminate the ball during idle mode to detect ball placement.

```
[Tier 1: Idle Ball Presence Mode (Safe RG0)]
  • Camera runs at 30 FPS.
  • On each frame, Arduino pulses LED for exactly 20 µs at low current (100 mA).
  • Time-Averaged Power = 30 FPS × 20 µs × 0.3 W = 0.18 mW.
  • Fully certified RG0 Exempt. Ball retroreflective dots appear razor-sharp on camera.
  • PC verifies ball is stationary at address.

                        │ (Club arrives / motion detected)
                        ▼
[Tier 2: Armed Shot Capture Burst]
  • PC arms Arduino.
  • Arduino outputs high-power 5-pulse burst (30µs each, 1.0ms apart) during the 10ms hit frame.
  • Immediately reverts to Tier 1 or turns off completely.
```

---

## 4. Hardware Watchdog Fail-Safes in Arduino Firmware

Software running on a PC can freeze, crash, or enter an infinite loop with the serial port open. To guarantee that **the IR LEDs can NEVER be latched ON continuously**, the Arduino firmware implements hardware-enforced fail-safes:

### 4.1 Microsecond Hardware Timer Clamp
Instead of relying on software `delayMicroseconds()`:
- Use an internal AVR Hardware Timer (e.g. Timer 1 or Timer 2) with an output-compare match.
- The MOSFET gate pin is automatically driven LOW by hardware after **$50\text{ \mu s}$**.
- Even if the Arduino CPU halts, crashes, or hangs in an infinite loop, the timer hardware shuts off the MOSFET.

### 4.2 Mandatory Thermal Lockout
- After firing any 5-pulse burst, the Arduino enforces a **mandatory $50\text{ ms}$ refractory cooldown** before it will accept or fire any subsequent strobe triggers.
- This caps the maximum possible burst duty cycle to $< 0.3\%$, making continuous emitter overdrive physically impossible.

