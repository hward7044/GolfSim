#pragma once

/**
 * @brief Operating power mode for the 850nm stroboscopic emitter.
 */
enum class EmitterPowerMode {
    HIGH_STROBE_READY, // 300 Hz active stroboscopic mode (ball locked at address)
    LOW_STANDBY        // 10 Hz standby emitter protection mode (tee empty)
};

