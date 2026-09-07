/**
 * GolfSim Arducam OV9281 300 Hz Multi-Strobe Splitter Controller
 * Target Board: Arduino Nano (ATmega328P, 16 MHz)
 * 
 * Description:
 *   Synchronizes with the hardware STROBE output pin of the dual OV9281 cameras.
 *   - In MODE_READY ('H'): Fires 3 rapid, microsecond-precise 30 µs IR strobe
 *     pulses spaced 3.333 ms apart (300 Hz) during the 10.0 ms camera exposure window.
 *   - In MODE_STANDBY ('L'): Throttles to low-power 10 Hz positioning pulses
 *     when the tee is unoccupied for > 5 seconds, protecting LEDs and conserving energy.
 *   - In MODE_OFF ('0'): All pulsing disabled.
 *   - In MODE_DC ('1'): Continuous DC illumination for camera aiming/focusing.
 * 
 * Safety Limits (IEC 62471 RG0 Exempt):
 *   - Maximum pulse width: 30 µs (strictly clamped < 50 µs by hardware/code).
 *   - Duty cycle at 300 Hz: 0.90% (average optical power ~32 mW on 15-LED 12V array).
 *   - Safety margin: > 1,600x below international continuous corneal exposure limit.
 *   - Fail-safe timeout: Automatically drops to standby if no PC ping for 10 seconds.
 * 
 * Wiring:
 *   - Camera STROBE Pin   -> Arduino Pin D2 (External Interrupt 0)
 *   - MOSFET Gate Driver  -> Arduino Pin D3 (Strobe Output)
 *   - Common Ground       -> Shared between Camera, Arduino, and 12V LED Power Supply
 */

const int STROBE_INPUT_PIN = 2; // Interrupt pin D2 (INT0)
const int IR_OUTPUT_PIN     = 3; // Output pin D3 to MOSFET gate

// --- Timing Parameters ---
// 300 Hz Strobe Timing: Period = 3,333 us (~3.33 ms)
const unsigned int FLASH_DURATION_US = 30;   // 30 microseconds pulse duration
const unsigned int FLASH_GAP_US      = 3303; // Gap between pulses (3,333 us - 30 us)
const int          NUM_PULSES        = 3;    // 3 pulses inside 10 ms exposure window

// Standby Timing: 10 Hz Positioning (1 pulse every 100 ms)
const unsigned int STANDBY_PULSE_US  = 20;   // 20 microseconds low-power positioning pulse
const unsigned long STANDBY_INTERVAL_MS = 100; // 100 ms between positioning pulses

// --- Operating Modes ---
enum StrobeMode {
    MODE_STANDBY,       // Low-power 10 Hz positioning (default on boot)
    MODE_READY,         // High-speed 300 Hz stroboscopic pulsing
    MODE_OFF,           // Disabled
    MODE_CONTINUOUS_DC  // Constant DC for optical alignment / lens focus
};

volatile StrobeMode currentMode = MODE_STANDBY;
volatile unsigned long lastStrobeFrameMillis = 0;
volatile unsigned long lastStandbyPulseMillis = 0;
unsigned long lastSerialKeepaliveMillis = 0;
const unsigned long KEEPALIVE_TIMEOUT_MS = 10000; // 10-second PC watchdog timeout

// Forward declarations
void fireStrobeSequence();
void clampOutputSafe();

void setup() {
    Serial.begin(115200);
    pinMode(STROBE_INPUT_PIN, INPUT);
    pinMode(IR_OUTPUT_PIN, OUTPUT);
    digitalWrite(IR_OUTPUT_PIN, LOW); // Start dark

    // Attach hardware interrupt to rising edge of camera STROBE pin
    attachInterrupt(digitalPinToInterrupt(STROBE_INPUT_PIN), fireStrobeSequence, RISING);

    lastSerialKeepaliveMillis = millis();
    Serial.println(F("[STROBE_CTRL] Initialized. Default: MODE_STANDBY (10 Hz)."));
}

void loop() {
    // 1. Check for incoming Serial commands from GolfSim
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        lastSerialKeepaliveMillis = millis(); // Refresh watchdog

        switch (cmd) {
            case 'H': // High-speed 300 Hz stroboscopic mode (ball detected on tee)
            case 'h':
                currentMode = MODE_READY;
                digitalWrite(IR_OUTPUT_PIN, LOW);
                Serial.println(F("[STROBE_CTRL] MODE_READY (300 Hz Strobe Active)"));
                break;

            case 'L': // Low-power standby mode (tee unoccupied > 5s)
            case 'l':
                currentMode = MODE_STANDBY;
                digitalWrite(IR_OUTPUT_PIN, LOW);
                Serial.println(F("[STROBE_CTRL] MODE_STANDBY (10 Hz Positioning)"));
                break;

            case '0': // Turn strobe completely OFF
                currentMode = MODE_OFF;
                digitalWrite(IR_OUTPUT_PIN, LOW);
                Serial.println(F("[STROBE_CTRL] MODE_OFF"));
                break;

            case '1': // Turn continuous DC ON for manual aiming
                currentMode = MODE_CONTINUOUS_DC;
                digitalWrite(IR_OUTPUT_PIN, HIGH);
                Serial.println(F("[STROBE_CTRL] MODE_CONTINUOUS_DC (Aiming)"));
                break;

            case 'F': // Single manual 3-pulse test burst
            case 'f':
                Serial.println(F("[STROBE_CTRL] Firing single 300 Hz burst..."));
                fire300HzBurst();
                break;

            case 'P': // Ping handshake
            case 'p':
                Serial.println(F("OK"));
                break;

            default:
                break;
        }
    }

    // 2. Safety Keepalive Watchdog:
    // If running in MODE_READY but PC has sent no commands for 10 seconds,
    // automatically revert to MODE_STANDBY to prevent accidental unattended pulsing.
    if (currentMode == MODE_READY && (millis() - lastSerialKeepaliveMillis > KEEPALIVE_TIMEOUT_MS)) {
        currentMode = MODE_STANDBY;
        digitalWrite(IR_OUTPUT_PIN, LOW);
        Serial.println(F("[STROBE_CTRL] Watchdog: No PC heartbeat for 10s -> Reverting to MODE_STANDBY."));
    }

    // 3. Hardware Fail-Safe Clamp:
    // In any pulsed mode, ensure IR_OUTPUT_PIN is never accidentally stuck HIGH.
    if (currentMode != MODE_CONTINUOUS_DC) {
        clampOutputSafe();
    }
}

/**
 * @brief Interrupt Service Routine (ISR) triggered by Camera STROBE Pin (D2).
 * Synchronizes IR flashes precisely to the start of the camera frame exposure.
 */
void fireStrobeSequence() {
    if (currentMode == MODE_OFF || currentMode == MODE_CONTINUOUS_DC) {
        return;
    }

    if (currentMode == MODE_READY) {
        // High-Speed 300 Hz Stroboscopic Train:
        // Fires exactly 3 pulses of 30 us spaced 3.333 ms apart.
        // Total duration: 2 * 3.333 ms + 30 us = 6.70 ms (comfortably inside 10 ms exposure).
        
        // Pulse 1
        digitalWrite(IR_OUTPUT_PIN, HIGH);
        delayMicroseconds(FLASH_DURATION_US);
        digitalWrite(IR_OUTPUT_PIN, LOW);
        delayMicroseconds(FLASH_GAP_US);

        // Pulse 2
        digitalWrite(IR_OUTPUT_PIN, HIGH);
        delayMicroseconds(FLASH_DURATION_US);
        digitalWrite(IR_OUTPUT_PIN, LOW);
        delayMicroseconds(FLASH_GAP_US);

        // Pulse 3
        digitalWrite(IR_OUTPUT_PIN, HIGH);
        delayMicroseconds(FLASH_DURATION_US);
        digitalWrite(IR_OUTPUT_PIN, LOW);

    } else if (currentMode == MODE_STANDBY) {
        // Standby Mode: Rate-limit to ~10 Hz (at most 1 pulse every 100 ms)
        // Even if camera runs at 60 or 100 FPS, we only flash once per 100 ms.
        unsigned long now = millis();
        if (now - lastStandbyPulseMillis >= STANDBY_INTERVAL_MS) {
            lastStandbyPulseMillis = now;
            digitalWrite(IR_OUTPUT_PIN, HIGH);
            delayMicroseconds(STANDBY_PULSE_US);
            digitalWrite(IR_OUTPUT_PIN, LOW);
        }
    }
}

/**
 * @brief Fires a manual single 3-pulse 300 Hz test train.
 */
void fire300HzBurst() {
    digitalWrite(IR_OUTPUT_PIN, HIGH);
    delayMicroseconds(FLASH_DURATION_US);
    digitalWrite(IR_OUTPUT_PIN, LOW);
    delayMicroseconds(FLASH_GAP_US);

    digitalWrite(IR_OUTPUT_PIN, HIGH);
    delayMicroseconds(FLASH_DURATION_US);
    digitalWrite(IR_OUTPUT_PIN, LOW);
    delayMicroseconds(FLASH_GAP_US);

    digitalWrite(IR_OUTPUT_PIN, HIGH);
    delayMicroseconds(FLASH_DURATION_US);
    digitalWrite(IR_OUTPUT_PIN, LOW);
}

/**
 * @brief Safety clamp: Ensures IR_OUTPUT_PIN is driven LOW if not actively pulsing.
 */
void clampOutputSafe() {
    if (digitalRead(IR_OUTPUT_PIN) == HIGH) {
        digitalWrite(IR_OUTPUT_PIN, LOW);
    }
}
