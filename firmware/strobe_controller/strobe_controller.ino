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
    Serial.begin(115200); // Initialize Serial for manual PC triggers
    pinMode(STROBE_INPUT_PIN, INPUT);
    pinMode(IR_OUTPUT_PIN, OUTPUT);
    digitalWrite(IR_OUTPUT_PIN, LOW); // Start with emitter off

    // Attach interrupt to execute immediately on the rising edge of the STROBE pin
    attachInterrupt(digitalPinToInterrupt(STROBE_INPUT_PIN), fireStrobeSequence, RISING);
}

bool continuousMode = false;

void loop() {
    // Check if the PC sent a command via Serial
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'F' || cmd == 'f') {
            // Sustained burst (300ms) for visual live-stream debugging
            for (int i = 0; i < 150; ++i) {
                digitalWrite(IR_OUTPUT_PIN, HIGH);
                delayMicroseconds(FLASH_DURATION_US);
                digitalWrite(IR_OUTPUT_PIN, LOW);
                delayMicroseconds(FLASH_GAP_US);
            }
            if (!continuousMode) {
                digitalWrite(IR_OUTPUT_PIN, LOW);
            } else {
                digitalWrite(IR_OUTPUT_PIN, HIGH);
            }
        } else if (cmd == '1') {
            continuousMode = true;
            digitalWrite(IR_OUTPUT_PIN, HIGH);
        } else if (cmd == '0') {
            continuousMode = false;
            digitalWrite(IR_OUTPUT_PIN, LOW);
        }
    }
}

// Interrupt Service Routine (ISR)
// Microsecond single-frame pulse sequence (for hardware trigger)
void fireStrobeSequence() {
    if (continuousMode) return; // Ignore if continuously ON

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
