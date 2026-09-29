#pragma once
#include "Camera/CameraConfig.hpp"
#include "Math/DotClusterFinder.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <string>

/// Stereo pairing gates for StereoBallTrackerTrigger. Wide by design: with the
/// dot-cluster finder already rejecting reflections, these only have to keep
/// the one ball's left/right views paired with each other.
struct StereoConfig {
    double epipolarTolerancePx = 150.0;  ///< Max |yL - yR| after rectification (measured rig offset ~85 px)
    double disparityMinPx      = 5.0;    ///< Min xL - xR
    double disparityMaxPx      = 600.0;  ///< Max xL - xR
    bool   swapCameras         = false;  ///< Registered roles are physically reversed (what --swap-cameras does)

    static StereoConfig fromJson(const nlohmann::json& j, const StereoConfig& base = StereoConfig());
    nlohmann::json toJson() const;
};

/// IR strobe drive, sent to the controller at startup. The envelope constants
/// are the eye-safety budget from docs/refactor/05: the firmware clamps to the
/// same numbers independently, so neither side can talk the other past them.
struct StrobeConfig {
    int pulseWidthUs = 30;   ///< LED on-time per pulse. Wider = brighter per pulse but more motion blur

    static constexpr int    kRateHz          = 300;   ///< Pulse rate in the active tier (fixed: the kinematics assume it)
    static constexpr int    kMinPulseWidthUs = 10;
    static constexpr int    kMaxPulseWidthUs = 100;   ///< ~4.5 mm blur at 100 mph, the most the dot aspect filter tolerates
    static constexpr double kMaxDutyCycle    = 0.03;  ///< 3%: 180 mW average on the 6 W array, 8x under the RG0 devices in refactor 05
    /// Optical budget the firmware enforces on its own (MAX_AVG_PERMILLE):
    /// duty x pulse-current overdrive <= 16.7%, ~1 W average on the 6 W array.
    /// The viewer's bench mode may run stationary tests up to this, past the
    /// blur cap above, to preview brighter pulses at today's LED current.
    static constexpr double kMaxAvgOpticalFraction = 0.167;

    /// Fraction of time the LEDs are on in the active tier.
    double dutyCycle() const noexcept { return pulseWidthUs * 1e-6 * kRateHz; }

    /// Widest pulse the firmware's optical rule allows at a given overdrive ratio
    /// (556 us at 1x, 111 us at 5x). Bench tests only; shots stay at kMaxPulseWidthUs.
    static int benchPulseWidthLimitUs(double overdrive) noexcept {
        return static_cast<int>(kMaxAvgOpticalFraction * 1e6 / kRateHz / (overdrive > 0.0 ? overdrive : 1.0));
    }

    /// Motion blur of a 100 mph (44.7 m/s) ball over one pulse, mm.
    static double blurMmAt100mph(int widthUs) noexcept { return 44.7e-3 * widthUs; }

    /// Clamp pulseWidthUs into the envelope. Returns true if anything changed.
    bool clampToEnvelope();

    static StrobeConfig fromJson(const nlohmann::json& j, const StrobeConfig& base = StrobeConfig());
    nlohmann::json toJson() const;
};

/**
 * @brief Application configuration: compiled defaults, overridden by
 * config/golfsim.json, overridden by command-line flags (in main.cpp).
 *
 * First slice of the AppConfig planned in docs/refactor/06; grows from here.
 */
struct AppConfig {
    CameraConfig     camera;
    DotClusterConfig detector;
    StereoConfig     stereo;
    StrobeConfig     strobe;
    std::string      sourcePath;   ///< File the values came from; empty = compiled defaults

    /// Config file location, relative to the project root.
    static constexpr const char* kDefaultPath = "config/golfsim.json";

    /// Nearest directory at or above `start` that holds kDefaultPath, as an
    /// absolute path; empty if none does. That file is what marks the root.
    static std::string findProjectRootAbove(const std::filesystem::path& start);

    /// Project root as seen from this process: the first of the working
    /// directory and the executable's directory whose ancestors hold
    /// kDefaultPath. Anchoring paths on this makes the binary behave the same
    /// from the repo root, from build/, or from an IDE. Empty if not found.
    static std::string findProjectRoot();

    static AppConfig fromJson(const nlohmann::json& j, const AppConfig& base = AppConfig());
    nlohmann::json toJson() const;

    /// Load `path` over the compiled defaults. A missing file is not an error
    /// (defaults are returned and `loaded` is false); a malformed file logs an
    /// error and also falls back to defaults.
    static AppConfig loadFromFile(const std::string& path, bool* loaded = nullptr);

    /// One-line summary for the startup log.
    std::string describe() const;
};
