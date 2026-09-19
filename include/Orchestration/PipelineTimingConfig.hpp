#pragma once

/**
 * @brief Centralized timing, optical geometry, and stroboscopic configuration
 *        for the launch monitor processing pipeline.
 *
 * Configured by default for the 3.0 ft (0.9144 m / 914.4 mm) working distance
 * launch monitor geometry using dual OV9281 monochrome global shutter cameras
 * with 2.8mm low-distortion M12 lenses.
 */
struct PipelineTimingConfig {
    /// Nominal working distance from camera lens plane to tee/flight line (meters).
    /// At 3.0 ft: 0.9144 m (914.4 mm).
    double workingDistanceMeters = 0.9144;

    /// Interval between successive stroboscopic sub-pulses (milliseconds).
    /// At 300 Hz: 3.3333 ms (~3,333 microseconds).
    double pulseIntervalMs       = 3.3333;

    /// Minimum number of 3D ball centroids required to solve kinematics.
    /// Default: 3 points for linear regression & Procrustes SVD spin fit.
    int    minPointsToSolve      = 3;

    /// Maximum camera frames to accumulate for a single shot before forcing solve.
    /// Default: 2 frames (covers irons, wedges, and putts at 100 FPS).
    int    maxFramesPerShot      = 2;

    /// Number of consecutive empty frames after seeing ball pulses before triggering immediate solve.
    /// Default: 1 (solves immediately on 1st empty frame, eliminating latency).
    int    emptyFrameTimeout     = 1;

    /// Duration of each individual IR LED flash pulse (microseconds).
    /// Default: 30 us (freezes motion blur at 100 mph to under 1.4 mm).
    double subPulseDurationUs    = 30.0;

    /// Nominal apparent ball radius on the 1280x800 sensor at working distance (pixels).
    /// At 3.0 ft with 2.8mm lens (fx ~ 1000 px): ~23.3 px (diameter ~46.7 px).
    double nominalBallRadiusPx   = 23.3;

    /// Active continuous stroboscopic frequency (Hz) when ball is present on the tee.
    double highStrobeRateHz      = 300.0;

    /// Emitter protection standby stroboscopic frequency (Hz) when tee is empty.
    double standbyStrobeRateHz   = 10.0;

    /// Camera hardware exposure duration in microseconds.
    /// Default: 7,812 us — the smallest UVC log2 exposure step that holds all 3
    /// pulses of a 300 Hz train (6,696 us) and still fits the 10 ms frame period.
    /// Set from CameraConfig::exposureUs at startup (refactor 09).
    int    cameraExposureUs      = 7812;

    /// Camera frame rate the exposure window must fit inside (Hz).
    double cameraFrameRateHz     = 100.0;

    /// Expected number of stroboscopic pulses captured within each camera exposure window.
    int    strobePulseCount      = 3;

    /// Inactivity duration (seconds) with no ball on the tee before dropping to standby rate.
    double ballLossTimeoutSec    = 5.0;

    /// Helper to update strobe rate dynamically
    void setStrobeRateHz(double hz) noexcept {
        highStrobeRateHz = hz;
        if (hz > 0.0) {
            pulseIntervalMs = 1000.0 / hz;
        }
    }

    /// Duration of the full strobe train (first pulse start to last pulse end), microseconds.
    double strobeTrainDurationUs() const noexcept {
        if (strobePulseCount <= 0) return 0.0;
        return (strobePulseCount - 1) * (pulseIntervalMs * 1000.0) + subPulseDurationUs;
    }

    /// Frame period implied by cameraFrameRateHz, microseconds (0 if unset).
    double framePeriodUs() const noexcept {
        return cameraFrameRateHz > 0.0 ? 1e6 / cameraFrameRateHz : 0.0;
    }

    /// Validates that the exposure window holds every sub-pulse and still fits
    /// inside one frame period at the configured frame rate.
    bool isValidTiming() const noexcept {
        if (cameraExposureUs <= 0 || pulseIntervalMs <= 0.0 || strobePulseCount <= 0) {
            return false;
        }
        if (cameraExposureUs < static_cast<int>(strobeTrainDurationUs())) {
            return false;
        }
        if (cameraFrameRateHz > 0.0 && cameraExposureUs >= static_cast<int>(framePeriodUs())) {
            return false;
        }
        return true;
    }
};
