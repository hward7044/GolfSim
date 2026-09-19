#pragma once
#include <nlohmann/json.hpp>

/**
 * @brief Hardware camera settings applied to both OV9281 nodes at startup.
 *
 * This is the single source of truth for what the sensor is asked to do. The
 * UVC exposure control only offers power-of-two steps (see quantiseExposureUs),
 * so the value actually applied can differ from exposureUs; drivers report the
 * applied value back through IUsbVideoDriver::getHardwareExposureUs().
 *
 * Defaults hold the full 3-pulse 300 Hz strobe train (6696 us) inside the
 * exposure window at the smallest UVC step that fits, and keep the black level
 * at zero so the background stays dark for dots-only imaging
 * (docs/refactor/09_Exposure_Gain_Config_And_Dot_Cluster_Detection.md).
 */
struct CameraConfig {
    int  exposureUs   = 7812;   ///< Requested exposure; snapped to the nearest UVC log2 step
    int  gain         = 0;      ///< 0..100 UVC gain units
    int  brightness   = 0;      ///< Additive black level; keep at 0 for dots-only imaging
    int  targetFps    = 100;    ///< Media type to request. The Arducam OV9281 advertises 1280x800
                                ///< NV12 at 100 and 120 fps; what the stereo pair sustains has to
                                ///< be measured in a Release build (see refactor 09, 2.5).
    bool autoExposure = false;  ///< Always off in production
    bool autoGain     = false;

    /// Merge fields present in `j` over `base`; fields absent in `j` keep base values.
    static CameraConfig fromJson(const nlohmann::json& j, const CameraConfig& base = CameraConfig());
    nlohmann::json toJson() const;

    /// Clamp gain/brightness into their UVC ranges. Returns true if anything changed.
    bool clampToHardwareRanges();

    /// Frame period implied by targetFps, in microseconds (0 if targetFps <= 0).
    int framePeriodUs() const noexcept {
        return targetFps > 0 ? static_cast<int>(1000000.0 / targetFps) : 0;
    }

    // UVC CameraControl_Exposure is log2(seconds): -13 = 122 us ... -5 = 31250 us.
    static constexpr int kMinExposureLog2 = -13;
    static constexpr int kMaxExposureLog2 = -1;
    static constexpr int kMaxGain         = 100;
    static constexpr int kMaxBrightness   = 64;

    /// Round-to-nearest UVC log2 exposure step for a requested duration in microseconds.
    static int exposureUsToLog2(int microseconds) noexcept;
    /// Exposure duration in microseconds for a UVC log2 step.
    static int exposureLog2ToUs(int log2Value) noexcept;
    /// The exposure the hardware will actually use for a request, in microseconds.
    static int quantiseExposureUs(int microseconds) noexcept {
        return exposureLog2ToUs(exposureUsToLog2(microseconds));
    }
};
