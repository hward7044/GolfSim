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
    /// Default: 1.0 ms (1,000 microseconds).
    double pulseIntervalMs       = 1.0;

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
};
