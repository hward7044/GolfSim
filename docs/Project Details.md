================================================================================
PROJECT: DIY STEREOSCOPIC GOLF LAUNCH MONITOR (C++20) — Rev. 2
TARGET: OPEN GOLF SIMULATOR INTEGRATION
================================================================================

CHANGELOG FROM REV. 1:
The original design sampled ball spin by comparing two discrete video frames
captured at the camera's native frame rate. At realistic sensor frame rates
(100–210 fps depending on ROI), that approach aliases badly for wedge-range
spin (8,000–10,000+ RPM) — the ball can rotate 270°+ between frames, far past
the point where the SVD/Procrustes fit returns a meaningful answer. Rev. 2
replaces frame-rate-limited sampling with a STROBOSCOPIC MULTI-PULSE
SINGLE-EXPOSURE capture: one long camera exposure containing several distinct,
precisely-timed sub-flashes, so temporal resolution is set by the LED driver's
switching speed instead of USB/UVC readout. It also replaces full-ball marking
with SMALL RETROREFLECTIVE DOTS, and moves the IR emitters from a shared board
between the cameras to CLUSTERS MOUNTED AT EACH LENS, since retroreflective
gain depends heavily on keeping the light source and the lens on nearly the
same optical axis.

================================================================================

1. CORE SYSTEM ARCHITECTURE & ENVIRONMENT
--------------------------------------------------------------------------------
- Language: C++20 (Strictly multi-threaded, lock-free architecture).
- Deployment: Windows (Development) / Linux Desktop via Valve Proton (Production).
- Compute: Single-PC Monolithic architecture. C++ tracks in background; GPU renders sim.
- Physical Footprint: 12-foot deep room. Cameras mounted on the floor, 3.5 to 4.0 feet parallel to the tee.
- Illumination Philosophy (Rev. 2): Low duty-cycle, wide-angle diffuse 850nm IR, distributed across multiple modest-power emitters rather than concentrated in one or two overdriven ones. A "shot" is a rare event (one brief burst every 10–30s of practice), so time-averaged eye exposure stays far below what continuously-run commercial 850nm security illuminators already run at safely — but rely on components that are already rated for continuous indoor proximity use, and get the pulsing as extra margin, not as the only margin.

2. HARDWARE & SENSING CONFIGURATION
--------------------------------------------------------------------------------

2.1 CAMERAS & LENSES (unchanged)
- Sensor: Dual Arducam/OmniVision OV9281 (1MP Monochrome Global Shutter). No IR-cut filter.
- Lenses: 2.8mm focal length M12 low-distortion lenses yielding a ~56-64 inch horizontal FOV window at working distance.
- Baseline: Sensors mounted EXACTLY 100mm apart (center-to-center) on a rigid rail.
- Confirmed real spec ceiling: 100–120 fps MJPG at full 1280×800, up to ~210 fps at reduced 640×400/320×240 ROI over USB2.0 UVC. This is no longer the binding constraint for spin (see §8) but still governs idle-mode gate-check streaming and position sampling between shots.
- Sensor supports a genuine hardware external-trigger mode (documented Arducam feature): a rising edge on the trigger line outputs a frame, with a sleep state for low idle power. Use the documented trigger pin/mode rather than improvising raw register access over UVC XU transfers.

2.2 ILLUMINATION SYSTEM (redesigned)
- Placement: Small IR emitter cluster mounted directly around/adjacent to EACH camera lens — not a single shared board centered between the two cameras. Retroreflective return brightness falls off fast with divergence angle between the illumination axis and the camera's viewing axis (loses roughly an order of magnitude per 1° of divergence in the worst case, unusable past ~10°); a shared center board puts you at ~1–2.5° divergence for both cameras, a coaxial cluster keeps each camera under ~0.5°.
- Component Class: Continuous-rated 850nm indoor illuminator boards (e.g., small-form-factor ~3W boards sold for close-range camera night vision, current-adjustable), not hobbyist 5mm indicator LEDs — those lack both the wavelength margin and the peak output needed for a ~30µs sub-exposure.
- Drive Circuit: MOSFET switching stage plus a CAPACITOR BANK placed close to the LED array, discharged through the MOSFET per pulse — lets you draw well above the LEDs' continuous rating for a ~30µs pulse without relying on the power supply's transient response, and without running the LEDs hot between shots (duty cycle is a fraction of a percent).
- Beam Shape: Wide-angle diffuse (60–90°), not a collimated spot — spreads total power over more area, which both improves illumination uniformity across the capture volume and lowers peak radiance (the quantity that drives the retinal thermal hazard) compared to a narrow, focused beam of the same total output.
- Aiming: Low, along the ball's flight line, angled away from a standing golfer's eye height.
- NOTE: None of the above is a substitute for checking the finished assembly with an actual IR power meter before regular use — the above gets you a design with real safety margin stacked in from multiple directions (duty cycle, distributed power, diffuse angle, rated components), but a bench check is the only way to actually confirm a risk-group classification.

2.3 BALL MARKING (redesigned)
- No full-ball coating (aerodynamic/durability concerns are valid — a full retroreflective wrap or repaint is both a bigger flight disruption and a bigger wear problem than necessary).
- Instead: 3–4 SMALL RETROREFLECTIVE DOTS (~5–8mm, e.g. cut from 3M Scotchlite-type sheeting), placed roughly evenly around the ball — small enough to sit in the same disruption category as markings golfers already draw on balls routinely (alignment lines, logos), and matching documented precedent from patented marked-ball tracking systems, which use similarly small strips/spots rather than full coverage.
- Multiple dots (vs. one marker) solves two problems at once: (a) a single point can't fully constrain a 3D rotation and is prone to occlusion on the far side of the ball; (b) more dots improve the odds that at least one is glinting toward each camera on any given pulse.
- Treat marked balls as DEDICATED INSTRUMENTED BALLS — cheap range balls, reapplied periodically, not balls you're also trying to keep pristine for play. Budget for reapplying dots every so often; it's a low-cost, low-effort consumable, not a one-time setup.
- This marking is used ONLY for the spin/orientation layer. Ball position/centroid tracking still uses the ball's own (unmarked, diffuse) surface — see §6.

2.4 PULSE-TIMING MICROCONTROLLER (new)
- A dedicated MCU (ATtiny/STM32/Arduino Nano class) sits between the PC and the MOSFET driver.
- The PC's job on hit-detection is reduced to sending one "FIRE" command with burst parameters (pulse count N, spacing T) — it does NOT attempt to time individual sub-millisecond pulses itself. General-purpose OS thread scheduling has jitter in the hundreds-of-microseconds-to-low-milliseconds range, which is far too coarse for pulse spacing that needs to be accurate to tens of microseconds.
- The MCU generates the N-pulse train from a hardware timer and simultaneously drives the camera's external trigger line on the first pulse, so both cameras' exposure windows and the strobe sequence are locked together in hardware.

3. THE HAL (PRODUCER THREAD) — Always-Triggered MCU Flow
--------------------------------------------------------------------------------
- Camera State: Set to hardware external-trigger mode with a constant fixed exposure window (e.g. 10 ms) during initialization. Exposure/trigger mode is never changed on-the-fly.
- Idle Mode: The MCU triggers both cameras at a default low frame rate (e.g., 30 Hz) with 10 ms exposures. The IR strobe remains off (or fires a single low-power positioning pulse). The producer thread captures these frames continuously for the optical gate.
- On Hit Detection:
  1. The PC immediately transmits a "FIRE" command with burst parameters (pulse count N, spacing T) to the MCU over UART serial.
  2. Within the next scheduled trigger frame, the MCU fires the timed high-power N-pulse strobe train.
  3. Each camera returns ONE FRAME (optionally two, for redundancy) containing N distinct sharp ball images along the flight path.
- This bypasses all UVC/I2C register update latencies and VSYNC registration delays, maintaining a zero-latency hardware data-plane.
- NO MATH ALLOWED HERE — HAL only captures and enqueues.

4. THE ATOMIC RING BUFFER
--------------------------------------------------------------------------------
- Goal: Lock-free memory bridge between Producer (Hardware) and Consumer (Math).
- Structure: std::array<FrameSet, SIZE> where SIZE is a power of 2.
- Thread Safety: Use std::atomic<size_t> head and tail indices for non-blocking read/write synchronization using memory order fences.
- Memory Rule: PRE-ALLOCATE all cv::Mat arrays at boot. Do not call new, malloc, or resize inside the capture loop.
- Sizing (Rev. 2): Can shrink relative to Rev. 1 — you're now buffering one or two multi-exposure frames per shot rather than a fast continuous burst, so a much smaller ring (e.g. 4–8 slots) comfortably covers back-to-back shots with margin for retries.

5. OPTICAL GATE TRIGGER
--------------------------------------------------------------------------------
- Goal: Robust, environment-agnostic swing detection independent of sound or static tee placement.
- Setup: Define a fixed cv::Rect (Region of Interest) bounding box in the air 3 to 6 inches downrange from the physical tee location.
- Trigger Math: Run a lightweight cv::absdiff() exclusively on this small ROI box between Frame(N) and a static background reference frame to count non-zero pixel deltas.
- State Transition: When the ball crosses into the downrange box and pixel changes surpass MIN_BALL_PIXELS, the C++ producer instantly flags a hit. On confirmed hit, hands off to the HAL/MCU flow in §3 (UART "FIRE" command).
- Future Improvement: To combat ambient IR drift (e.g. slowly moving sun patches), future iterations should implement an exponential moving average (EMA) or multi-frame low-delta baseline refresh (`background = background * (1 - alpha) + current * alpha` with very small `alpha`) instead of a periodic static reference swap, preventing static shadow baking.

6. MULTI-BLOB 2D TRACKING (position + marker glints)
--------------------------------------------------------------------------------
Replaces the single-centroid-per-frame approach with per-frame, multi-blob
processing, since each captured frame now contains N ball images:

  1. SEGMENT BALL SILHOUETTES: Threshold at a level tuned to the ball's own diffuse IR return, findContours, filter by area/circularity to isolate N ball-sized blobs.
  2. ORDER BLOBS by position along the known flight-direction axis — motion is monotonic over the capture window, so this ordering is unambiguous and gives you correspondence across "frames" for free.
  3. PER-BLOB CENTROID: Same moments-based approach as Rev. 1 (x_c = m10/m00, y_c = m01/m00), run once per blob instead of once per frame.
  4. SEGMENT MARKER GLINTS within each ball blob: Run a second, much higher threshold restricted to each blob's bounding region — retroreflective dots return roughly an order of magnitude (or more, near zero divergence angle) more light than the ball's own diffuse surface, so they separate cleanly from the ball silhouette with a harder threshold rather than needing a different detection method.
  5. Record, per blob: 2D ball centroid + 2D positions of whichever marker dots are visible (expect 0–4 depending on rotation at that instant; 0 is fine occasionally, just skip that blob's contribution to the spin fit in §8).

- KNOWN EDGE CASE TO VALIDATE ON THE BENCH: At the high-spin/moderate-speed end (wedges), the pulse spacing needed to avoid rotational aliasing (~0.75–0.85ms, see §8) is tighter than the ball-diameter/ball-speed spacing needed to fully separate blobs (~1.0–1.1ms at typical wedge ball speed) — meaning blobs may be tangent or slightly overlapping at that extreme. This isn't fatal (touching-circle segmentation via Hough-circle fitting or watershed is a solved problem) but it does mean naive connected-component blob detection alone likely isn't sufficient for wedge shots specifically — plan on it rather than discovering it later.

7. 3D TRIANGULATION
--------------------------------------------------------------------------------
- Calibration and rectification pipeline unchanged (stereoCalibrate, stereoRectify).
- Now applied per-blob instead of once per shot: each of the N ordered ball-blob pairs (left/right camera) triangulates to one 3D position; each visible marker-dot pair triangulates to one 3D marker position.
- Output per shot: An N-point 3D position time series (ball speed and launch angles now come directly from this, via known inter-pulse Δt, rather than needing a separate slow-frame-rate track) plus an N-point (with occasional gaps) 3D marker position series feeding §8.

8. SPIN CALCULATION (Incremental Procrustes / Eigen SVD)
--------------------------------------------------------------------------------
Replaces the Rev. 1 two-frame fit with an N-sample incremental fit, which is
the actual fix for the aliasing problem identified during review:

  1. For each consecutive pulse pair (i, i+1) where at least 2 non-antipodal markers are visible in both, compute the incremental rotation via the same cross-covariance + Eigen::JacobiSVD Procrustes approach as Rev. 1 (H from centered marker sets → U, Σ, Vᵀ → R = V·Uᵀ).
  2. Because pulse spacing is chosen to keep inter-pulse rotation under ~45°, each incremental fit stays well clear of the aliasing/wraparound region that broke the two-frame approach.
  3. Least-squares fit a constant angular velocity and axis across all valid incremental rotations for the shot, rather than trusting any single pair — this also gives you a natural way to detect and discount a bad increment (e.g. from a marker misidentification) as an outlier.
  4. PULSE SPACING TARGET: T_max(ms) ≈ 7500 / RPM_expected. Concretely: driver-range spin (~2,500–3,000 RPM) tolerates ~2.5–3ms spacing; wedge-range spin (~8,000–10,000+ RPM) needs ~0.75–0.85ms spacing. These pulses are timed and generated by the MCU entirely within the fixed 10 ms camera integration window. A fixed default (e.g. 6 pulses @ ~1.2ms) is a reasonable MVP starting point covering mid-range clubs.
  5. Report RPM and axis exactly as Rev. 1: Angle = arccos((Trace(R)-1)/2), RPM = (Angle/Δt) × (60/2π); axis = eigenvector of R for eigenvalue 1 — the math is unchanged, only the sampling strategy that feeds it.

9. TCP TELEMETRY
--------------------------------------------------------------------------------
- Goal: Deliver payload to OpenGolfSim in < 15ms.
- Connection: Asynchronous TCP socket to 127.0.0.1 on Port 3111 (or 49152). Use Winsock2 (Windows) / sys/socket.h (Linux).
- Payload: Construct using nlohmann/json. 
- Keys Required: "type": "shot", "ballSpeed", "verticalLaunchAngle", "horizontalLaunchAngle", "spinSpeed", "spinAxis".
- Delivery: Convert to string (.dump()), append '\n', and send() over the stream.

10. DIAGNOSTICS
--------------------------------------------------------------------------------
- Global Logger: Use spdlog wrapped in a Singleton macro (LOG_INFO) for zero-overhead text debugging.
- Flight Recorder: Now saves one (or two) rich multi-exposure frames per shot instead of a large frame burst — meaningfully smaller storage footprint, and genuinely easier to debug visually, since a single saved image already shows the full stroboscopic ball trail rather than requiring a review tool to step through dozens of frames.

11. OPEN ENGINEERING RISKS / BENCH-TEST CHECKLIST
--------------------------------------------------------------------------------
Worth validating in roughly this order before writing much code against fixed
assumptions:

  1. SUB-PULSE EXPOSURE ADEQUACY: Confirm the chosen LED board, at your working distance, actually exposes the ball well within a single ~30µs pulse — this is the same brightness question as Rev. 1, just now needing to hold for N repeated pulses per exposure rather than once.
  2. RETROREFLECTIVE DOT VISIBILITY ANGLE: Empirically check how far off-axis (camera vs. ball-dot-normal) the dots stay usably bright — informs how many dots you actually need and where they should sit relative to typical ball orientations at address.
  3. MCU → CAMERA TRIGGER LATENCY & JITTER: Measure actual round-trip from FIRE command to first pulse and pulse-to-pulse spacing accuracy on the real hardware; this number feeds directly into the Δt used in the spin/speed math, so any systematic offset needs to be calibrated out.
  4. BLOB SEPARATION AT WEDGE-RANGE SPIN/SPEED (§6): Confirm whether tangent/overlapping blobs actually occur at your real spacing choice, and whether basic contour filtering is sufficient or you need circle-fitting/watershed segmentation.
  5. ILLUMINATION SAFETY CHECK: Point a handheld IR power meter at the finished, mounted assembly from typical head-height/distance before relying on it for regular sessions — the design choices in §2.2 stack real margin, but a bench check is the only way to confirm it.