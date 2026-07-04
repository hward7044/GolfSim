/**
 * Arducam OV9281 Multi-Strobe Splitter Controller
 * Target Board: Arduino Nano (or any ATmega328P board)
 * 
 * Description:
 *   Listens to the rising edge of the camera's hardware STROBE output pin,
 *   and immediately fires three rapid, microsecond-precise IR strobe pulses
 *   during that single frame's exposure window.
 * 
 * Wiring:
 *   - Camera STROBE Pin  -> Arduino Pin D2 (Interrupt 0)
 *   - Emitter MOSFET Gate -> Arduino Pin D3 (Strobe Output)
 *   - Camera/Arduino GND -> Common Ground
 */

const int STROBE_INPUT_PIN = 2; // Interrupt pin D2
const int IR_OUTPUT_PIN = 3;    // Output pin D3 to MOSFET gate

// Timing parameters (microseconds)
const int FLASH_DURATION_US = 100; // Duration of each IR flash (100 microseconds)
const int FLASH_GAP_US = 900;      // Delay between flashes (900 microseconds = 1ms interval)

void setup() {
    pinMode(STROBE_INPUT_PIN, INPUT);
    pinMode(IR_OUTPUT_PIN, OUTPUT);
    digitalWrite(IR_OUTPUT_PIN, LOW); // Start with emitter off

    // Attach interrupt to execute immediately on the rising edge of the STROBE pin
    attachInterrupt(digitalPinToInterrupt(STROBE_INPUT_PIN), fireStrobeSequence, RISING);
}

void loop() {
    // Keep loop empty. All microsecond strobe timing is handled by hardware interrupts.
}

// Interrupt Service Routine (ISR)
// Fires 3 quick strobe pulses spaced 1ms apart (from start of flash to start of flash)
void fireStrobeSequence() {
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
}
