#pragma once
#include "Camera/CameraConfig.hpp"
#include "Math/DotClusterFinder.hpp"
#include <nlohmann/json.hpp>
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
    std::string      sourcePath;   ///< File the values came from; empty = compiled defaults

    static constexpr const char* kDefaultPath = "config/golfsim.json";

    static AppConfig fromJson(const nlohmann::json& j, const AppConfig& base = AppConfig());
    nlohmann::json toJson() const;

    /// Load `path` over the compiled defaults. A missing file is not an error
    /// (defaults are returned and `loaded` is false); a malformed file logs an
    /// error and also falls back to defaults.
    static AppConfig loadFromFile(const std::string& path, bool* loaded = nullptr);

    /// One-line summary for the startup log.
    std::string describe() const;
};
