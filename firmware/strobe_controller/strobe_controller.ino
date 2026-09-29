/**
 * GolfSim 300 Hz IR Strobe Controller
 * Target Board: Arduino Nano (ATmega328P, 16 MHz)
 *
 * Description:
 *   Free-running IR strobe for the dual OV9281 cameras. The cameras and this
 *   controller are not synchronised: the PC only selects a tier over USB serial
 *   and the LEDs pulse continuously from Timer1, so every camera frame already
 *   holds a multi-pulse image when a shot happens (docs/refactor/05, section 3).
 *   - MODE_READY   ('H'): pulses at 300 Hz (every 3.333 ms). A 7.8 ms exposure
 *                         at 100 fps captures 2 complete pulses in every frame
 *                         and 3 in about a third of them.
 *   - MODE_STANDBY ('L'): pulses at 10 Hz, tee unoccupied. Default on boot.
 *   - MODE_OFF     ('0'): no pulsing.
 *   - MODE_AIM     ('1'): aiming / focusing. Hardware PWM on D3 (Timer2, 3.9 kHz)
 *                         at 1/LED_OVERDRIVE_RATIO duty, so the array averages
 *                         its rated current whatever the string resistors are.
 *                         At ratio 1 this is plain DC, as before.
 *   - 'F'               : one manual 3-pulse 300 Hz burst (test).
 *   - 'W<us>\n'         : pulse width in microseconds, e.g. "W60\n". Clamped to
 *                         the safety envelope below whatever the PC asks for and
 *                         echoed. Resets to 30 us on boot (the PC sends it at
 *                         startup). The 100 us motion-blur limit for shots is
 *                         the PC's (StrobeConfig); the viewer's bench mode goes
 *                         past it on purpose for stationary brightness tests.
 *   - 'P'               : replies "OK mode=<READY|STANDBY|OFF|DC> pulses=<n> width=<us> od=<ratio>",
 *                         n = pulses fired since boot. Also refreshes the watchdog.
 *
 * LED drive (LED_OVERDRIVE_RATIO):
 *   As built (2026-09): 15 x 3 W 850 nm COB LEDs (400-500 mA rated) in three
 *   5-LED strings on 12 V, mounted in rings around the camera lenses (10 around
 *   the right camera, 5 around the left) with a bulk capacitor across the
 *   array. With 10 ohm string resistors a string draws ~0.45 A: rated, in
 *   every mode. The brightness lever is pulse current: lower the resistors
 *   (~1 ohm for ~2.5 A) and set LED_OVERDRIVE_RATIO to I_pulse / I_rated
 *   (5.0). The ratio then fixes everything the LEDs need to survive it:
 *   aiming mode runs at 1/ratio duty, and the pulse width is capped so the
 *   average current stays well under rated. Flash this constant to match
 *   the board it drives.
 *
 * Timing:
 *   Timer1 runs in CTC mode at 300 Hz (16 MHz / 8 prescaler, 0.5 us ticks,
 *   OCR1A = 6666 -> 3333.5 us). The compare ISR raises D3, busy-waits the
 *   pulse width and drops it again, all with interrupts off, so a pulse can
 *   never be stretched or interrupted. Standby fires on every 30th tick.
 *
 * Safety Limits (IEC 62471 RG0 Exempt, docs/refactor/05):
 *   - Duty x LED_OVERDRIVE_RATIO <= 16.7%: average optical output stays under
 *     ~1 W (6 W array figure) however hard the pulses are driven; the
 *     baby-monitor class in docs/refactor/05 is 1.5 W. At ratio 1 that is a
 *     556 us pulse at 300 Hz, at ratio 5 it is 111 us. Also keeps the LEDs'
 *     average current far under rated.
 *   - Absolute width 10..1000 us as a sanity bound.
 *   Enforced here regardless of what arrives over serial.
 *   Default 30 us: 0.90% duty at 300 Hz, 0.03% at 10 Hz.
 *   - Watchdog: MODE_READY drops to MODE_STANDBY after 10 s without a byte
 *     from the PC. The PC therefore sends a byte ('P' is harmless) every few
 *     seconds while it wants 300 Hz held.
 *
 * Wiring:
 *   - Arduino Pin D3 (PD3) -> Logic-level MOSFET gate driving the IR LED array
 *   - Common Ground        -> Shared between Arduino and the 12 V LED supply
 *   (No camera connection: the earlier camera-STROBE -> D2 sync was dropped.)
 */

const int IR_OUTPUT_PIN = 3;                     // D3 = PORTD bit 3 on the Nano
#define IR_PORT     PORTD
#define IR_BIT      _BV(PD3)

// --- LED Drive ---
// Pulse current / rated continuous current for THIS board's string resistors:
//   10 ohm -> 1.0 (rated, as built)      2.2 ohm -> ~3.0      1 ohm -> ~5.0
const float         LED_OVERDRIVE_RATIO = 1.0f;

// --- Timing Parameters ---
const unsigned int  DEFAULT_PULSE_US    = 30;    // Pulse width on boot, every mode
const unsigned int  MIN_PULSE_WIDTH_US  = 10;
const unsigned int  MAX_PULSE_WIDTH_US  = 1000;  // Sanity bound; the optical rule below binds first
const unsigned int  MAX_AVG_PERMILLE    = 167;   // duty x overdrive: ~1 W average optical on the 6 W array
const unsigned int  STROBE_RATE_HZ      = 300;   // MODE_READY pulse rate
const unsigned int  STANDBY_DIVIDER     = 30;    // MODE_STANDBY = 300 Hz / 30 = 10 Hz
// Timer1 at 16 MHz / 8 = 2 MHz ticks; period = (TOP + 1) * 0.5 us.
// 300 Hz wants 6666.67 ticks: TOP 6666 -> 6667 ticks = 3333.5 us (+0.005%).
const unsigned int  TIMER1_TOP          = 6666;

// --- Operating Modes ---
enum StrobeMode {
    MODE_STANDBY,       // 10 Hz positioning pulses (default on boot)
    MODE_READY,         // 300 Hz stroboscopic pulsing
    MODE_OFF,           // Disabled
    MODE_CONTINUOUS_DC  // Aiming: PWM at rated average current (plain DC at ratio 1)
};

volatile StrobeMode    currentMode    = MODE_STANDBY;
volatile unsigned long pulseCount     = 0;       // Pulses fired since boot, all modes
volatile unsigned int  standbyDivider = 0;
volatile unsigned int  pulseWidthUs   = DEFAULT_PULSE_US;
// 'W' command parser: digits accumulate until a non-digit ends the number.
bool         readingWidth = false;
unsigned int pendingWidth = 0;

unsigned long lastSerialKeepaliveMillis = 0;
const unsigned long KEEPALIVE_TIMEOUT_MS = 10000; // 10-second PC watchdog timeout

// Forward declarations
void firePulse();
void fire300HzBurst();
void clampOutputSafe();
void startStrobeTimer();
void startAimingPwm();
void applyPulseWidth(unsigned int requestedUs);
const __FlashStringHelper* modeName(StrobeMode mode);

void setup() {
    Serial.begin(115200);
    pinMode(IR_OUTPUT_PIN, OUTPUT);
    digitalWrite(IR_OUTPUT_PIN, LOW); // Start dark

    startStrobeTimer();

    lastSerialKeepaliveMillis = millis();
    Serial.println(F("[STROBE_CTRL] Initialized. Default: MODE_STANDBY (10 Hz)."));
}

void loop() {
    // 1. Check for incoming Serial commands from GolfSim
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        lastSerialKeepaliveMillis = millis(); // Refresh watchdog

        // "W<digits>" in progress: consume digits, apply on the first non-digit.
        if (readingWidth) {
            if (cmd >= '0' && cmd <= '9') {
                if (pendingWidth < 10000) pendingWidth = pendingWidth * 10 + (cmd - '0');
                return;
            }
            readingWidth = false;
            applyPulseWidth(pendingWidth);
            if (cmd == '\n' || cmd == '\r') return;   // terminator consumed; anything else is a new command
        }

        switch (cmd) {
            case 'W': // Pulse width follows as decimal microseconds
            case 'w':
                readingWidth = true;
                pendingWidth = 0;
                break;

            case 'H': // 300 Hz stroboscopic mode (ball on tee)
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

            case '1': // Aiming: rated average current, never latched harder than that
                currentMode = MODE_CONTINUOUS_DC;
                startAimingPwm();
                Serial.print(F("[STROBE_CTRL] MODE_CONTINUOUS_DC (Aiming, "));
                Serial.print(100.0f / LED_OVERDRIVE_RATIO, 0);
                Serial.println(F("% duty)"));
                break;

            case 'F': // Single manual 3-pulse test burst
            case 'f':
                Serial.println(F("[STROBE_CTRL] Firing single 300 Hz burst..."));
                fire300HzBurst();
                break;

            case 'P': // Ping: keepalive + status
            case 'p': {
                unsigned long pulses;
                unsigned int width;
                noInterrupts();
                pulses = pulseCount;
                width  = pulseWidthUs;
                interrupts();
                Serial.print(F("OK mode="));
                Serial.print(modeName(currentMode));
                Serial.print(F(" pulses="));
                Serial.print(pulses);
                Serial.print(F(" width="));
                Serial.print(width);
                Serial.print(F(" od="));
                Serial.println(LED_OVERDRIVE_RATIO, 1);
                break;
            }

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

    // 3. Fail-Safe Clamp:
    // In any pulsed mode the pin must be LOW between pulses. A pulse completes
    // inside the timer ISR before loop() can run again, so this only ever
    // catches a pin left HIGH by a mode change or a fault.
    if (currentMode != MODE_CONTINUOUS_DC) {
        clampOutputSafe();
    }
}

/**
 * @brief Timer1 in CTC mode ticking at the 300 Hz strobe rate.
 */
void startStrobeTimer() {
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = TIMER1_TOP;
    TCCR1B |= _BV(WGM12);   // CTC, TOP = OCR1A
    TCCR1B |= _BV(CS11);    // prescaler 8 -> 0.5 us per tick
    TIMSK1 |= _BV(OCIE1A);  // compare-match interrupt
    interrupts();
}

/**
 * @brief 300 Hz tick. Fires the pulse in MODE_READY, every 30th tick in
 * MODE_STANDBY, and nothing otherwise. Runs with interrupts disabled.
 */
ISR(TIMER1_COMPA_vect) {
    if (currentMode == MODE_READY) {
        firePulse();
    } else if (currentMode == MODE_STANDBY) {
        if (++standbyDivider >= STANDBY_DIVIDER) {
            standbyDivider = 0;
            firePulse();
        }
    }
}

/**
 * @brief Aiming mode: hardware PWM on D3 (Timer2 OC2B) at 1/LED_OVERDRIVE_RATIO
 * duty. Timer2 is set to prescaler 8 (phase-correct, 3.9 kHz) so each on-time
 * is short against the die's thermal time constant. A hardware PWM cannot be
 * left stuck HIGH by a hung CPU. At ratio 1 analogWrite(255) is plain HIGH.
 */
void startAimingPwm() {
    TCCR2B = (TCCR2B & 0xF8) | _BV(CS21);   // prescaler 8
    int duty = (int)(255.0f / LED_OVERDRIVE_RATIO + 0.5f);
    if (duty > 255) duty = 255;
    if (duty < 1) duty = 1;
    analogWrite(IR_OUTPUT_PIN, duty);       // digitalWrite(LOW) elsewhere disconnects the PWM again
}

/**
 * @brief Clamp a requested pulse width into the safety envelope and apply it.
 * All bounds are enforced here, not trusted from the PC.
 */
void applyPulseWidth(unsigned int requestedUs) {
    // Average optical bound scales down with the pulse current: 556 us at ratio 1, 111 us at ratio 5.
    const unsigned int avgLimitUs = (unsigned int)(MAX_AVG_PERMILLE * 1000.0f / STROBE_RATE_HZ / LED_OVERDRIVE_RATIO);
    unsigned int limitUs = MAX_PULSE_WIDTH_US;
    if (avgLimitUs < limitUs) limitUs = avgLimitUs;
    unsigned int us = requestedUs;
    if (us < MIN_PULSE_WIDTH_US) us = MIN_PULSE_WIDTH_US;
    if (us > limitUs) us = limitUs;

    noInterrupts();
    pulseWidthUs = us;
    interrupts();

    Serial.print(F("[STROBE_CTRL] Pulse width "));
    Serial.print(us);
    Serial.print(F(" us ("));
    Serial.print((unsigned long)us * STROBE_RATE_HZ / 10000UL);   // duty in 0.1% steps
    Serial.print(F("/10 % duty at 300 Hz)"));
    if (us != requestedUs) {
        Serial.print(F(" - clamped from "));
        Serial.print(requestedUs);
    }
    Serial.println();
}

/**
 * @brief One pulse of pulseWidthUs on D3. Direct port writes: digitalWrite
 * costs several microseconds per call, which would stretch the pulse.
 */
void firePulse() {
    IR_PORT |= IR_BIT;
    delayMicroseconds(pulseWidthUs);
    IR_PORT &= ~IR_BIT;
    pulseCount++;
}

/**
 * @brief Fires a manual single 3-pulse 300 Hz test train.
 */
void fire300HzBurst() {
    const unsigned int gapUs = 1000000UL / STROBE_RATE_HZ - pulseWidthUs;
    noInterrupts();   // keep the free-running tick from adding a fourth pulse mid-burst
    firePulse();
    delayMicroseconds(gapUs);
    firePulse();
    delayMicroseconds(gapUs);
    firePulse();
    interrupts();
}

const __FlashStringHelper* modeName(StrobeMode mode) {
    switch (mode) {
        case MODE_READY:         return F("READY");
        case MODE_STANDBY:       return F("STANDBY");
        case MODE_OFF:           return F("OFF");
        case MODE_CONTINUOUS_DC: return F("DC");
    }
    return F("?");
}

/**
 * @brief Safety clamp: Ensures IR_OUTPUT_PIN is driven LOW if not actively pulsing.
 */
void clampOutputSafe() {
    if (digitalRead(IR_OUTPUT_PIN) == HIGH) {
        digitalWrite(IR_OUTPUT_PIN, LOW);
    }
}
