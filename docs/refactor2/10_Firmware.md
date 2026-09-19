# 10 — Firmware (`strobe_controller.ino`)

**Principle:** the code must implement the safety properties the design docs claim, and an ISR must be short.
**Effort:** Medium (one file, but needs a scope on the bench). **Risk:** **High** — the eye-safety argument in [05](../refactor/05_Shot_Detection_IR_Safety.md) rests on a mechanism that does not exist in the code, and the ISR design plausibly explains lost serial commands.

---

## Findings

### F1 — The 3-pulse train runs inside the ISR with interrupts disabled for 6.7 ms per frame
[strobe_controller.ino:139-171](../../firmware/strobe_controller/strobe_controller.ino#L139-L171): `fireStrobeSequence` is the `RISING` ISR on D2 and executes `delayMicroseconds(30) … (3333) … (30) … (3333) … (30)` inline. On AVR an ISR runs with the global interrupt flag cleared, so for **6.76 ms of every frame** nothing else can interrupt. At the design's 100 fps that is **67 % of wall time in MODE_READY**.

What that breaks:
- **`millis()` stalls.** Timer0's overflow interrupt (every 1.024 ms) is blocked ~6 times per frame and only one is serviced afterwards → `millis()` advances ≈ 3–4 ms per real 10 ms in MODE_READY. The 10 s serial watchdog is really ≈ 25–30 s; `STANDBY_INTERVAL_MS` is unaffected only because standby's ISR is short.
- **Serial bytes are lost.** At 115200 baud a byte arrives every 87 µs; the ATmega328P UART holds two before overrun. The core's RX ISR cannot run for 6.7 ms → any command sent by the PC while a train is firing is dropped with high probability. Combined with the PC sending `'H'`/`'L'` **once** (deduplicated — [04 S1](04_Single_Responsibility.md)), a lost byte is never retried. This is a concrete mechanism for "the strobe didn't switch modes".
- **The software clamp cannot run** during the train (see F2).

**Fix:** the ISR only *starts* the train; Timer1 does the rest.
```
ISR(D2 rising):  pulseIndex = 0; start Timer1 (CTC, OCR1A = 3333 µs); drive pin HIGH; arm OCR1B = 30 µs.
ISR(TIMER1_COMPB): pin LOW                            // pulse end, hardware-timed
ISR(TIMER1_COMPA): if (++pulseIndex < 3) pin HIGH;   // next pulse
                   else stop Timer1
```
Each ISR is a few microseconds; `millis()` and the UART keep working. Better still: use Timer1's **hardware output compare on OC1B (D10)** to toggle the pin so the LOW edge does not even need an ISR — move the MOSFET gate from D3 to D10.

### F2 — The documented hardware clamp is not implemented
[05 §4.1](../refactor/05_Shot_Detection_IR_Safety.md) and the refactor README state "Arduino Timer1 hardware 50 µs clamp" / "Hardware Timer CTC Driver with Microsecond Clamp". The firmware has **no Timer1 configuration at all**. The only clamp is [`clampOutputSafe()`](../../firmware/strobe_controller/strobe_controller.ino#L201): a `digitalRead` in `loop()` that drives the pin LOW if it finds it HIGH. That runs only when `loop()` gets CPU — never during the ISR — and not at all if the MCU hangs inside the ISR with the pin HIGH, which is exactly the failure the clamp is supposed to cover.

The eye-safety math in 05 (0.9 % duty cycle, 54 mW average) assumes the pulse can never exceed 50 µs. Today a crash between `digitalWrite(HIGH)` and `digitalWrite(LOW)` leaves the emitter on indefinitely.

**Fix:** (a) implement the Timer1 design from F1 — with hardware output compare the pin falls at OCR1B regardless of software state; (b) for a true fail-safe independent of firmware, add a hardware one-shot (555 monostable or RC + comparator) between D3/D10 and the MOSFET gate, sized to 50 µs, so **no** software fault can hold the LEDs on. Then update 05 to describe what is actually there.

### F3 — Pulse train duplicated
`fireStrobeSequence` (ISR) and `fire300HzBurst` (`'F'` test command) contain the same 15 lines ([139-190](../../firmware/strobe_controller/strobe_controller.ino#L139-L190)). Disappears with F1 (`'F'` just calls the same "start train" routine).

### F4 — Watchdog vs. deduplicated PC commands (known)
Firmware reverts to STANDBY after 10 s without a byte; the PC sends each mode byte once. Firmware side: fine as designed. PC side: the `StrobeController` from [04 S1](04_Single_Responsibility.md) sends a `'P'` ping every ≤ 5 s while in READY. Note F1 makes the effective timeout ≈ 25–30 s today, which has been masking this.

### F5 — No identity/config handshake
`'P'` answers `OK`. The PC cannot verify firmware version, pulse width, pulse count or spacing — the values `PipelineTimingConfig` ([06 C2](06_Configuration_And_Magic_Numbers.md)) and the kinematics ([07](07_Data_Model_And_Time_Base.md)) depend on. **Fix:** `'?'` → `GOLFSIM-STROBE v2 pulses=3 width_us=30 gap_us=3333 standby_hz=10`; the PC checks it against its config at start-up and refuses to run on mismatch.

---

## Plan

1. F1 + F3 — rewrite the pulse generation on Timer1 (~80 lines). Verify on a scope: 3 × 30 µs pulses at 3.333 ms, first edge ≤ 5 µs after STROBE; `millis()` drift ≤ 0.1 % over 60 s in READY; serial echo test (send 100 `'P'`s during READY, receive 100 `OK`s).
2. F2 — hardware one-shot on the gate line (bench work); then correct [05 §4.1](../refactor/05_Shot_Detection_IR_Safety.md).
3. F5 — `'?'` command + PC-side check.
4. F4 — PC-side ping (lives in [04 S1](04_Single_Responsibility.md)).
5. Host-side unit tests of the state machine with an Arduino shim (coverage plan §8.6) once F1 has made the logic separable from the timing.

## Verification
- Scope captures for pulse width, spacing, and worst-case width with the firmware deliberately halted (`while(1);` injected after `HIGH`) — the one-shot must still cut at 50 µs.
- Serial reliability: 0 lost commands in 10 000 during READY at 100 fps.
- `millis()` vs PC clock over 5 minutes in READY: < 0.1 % drift.

**Depends on:** nothing. **Feeds:** 04 S1 (keep-alive), 06 C2 (handshake), 07 (timing contract).
