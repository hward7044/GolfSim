#include "App/AppConfig.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <spdlog/spdlog.h>

// =============================================================================
// StereoConfig
// =============================================================================

StereoConfig StereoConfig::fromJson(const nlohmann::json& j, const StereoConfig& base) {
    StereoConfig cfg = base;
    if (!j.is_object()) return cfg;
    cfg.epipolarTolerancePx = j.value("epipolarTolerancePx", cfg.epipolarTolerancePx);
    cfg.disparityMinPx      = j.value("disparityMinPx",      cfg.disparityMinPx);
    cfg.disparityMaxPx      = j.value("disparityMaxPx",      cfg.disparityMaxPx);
    cfg.swapCameras         = j.value("swapCameras",         cfg.swapCameras);
    return cfg;
}

nlohmann::json StereoConfig::toJson() const {
    return {
        {"epipolarTolerancePx", epipolarTolerancePx},
        {"disparityMinPx",      disparityMinPx},
        {"disparityMaxPx",      disparityMaxPx},
        {"swapCameras",         swapCameras},
    };
}

// =============================================================================
// AppConfig
// =============================================================================

AppConfig AppConfig::fromJson(const nlohmann::json& j, const AppConfig& base) {
    AppConfig cfg = base;
    if (!j.is_object()) return cfg;
    if (j.contains("camera"))   cfg.camera   = CameraConfig::fromJson(j["camera"], cfg.camera);
    if (j.contains("detector")) cfg.detector = DotClusterConfig::fromJson(j["detector"], cfg.detector);
    if (j.contains("stereo"))   cfg.stereo   = StereoConfig::fromJson(j["stereo"], cfg.stereo);
    return cfg;
}

nlohmann::json AppConfig::toJson() const {
    return {
        {"camera",   camera.toJson()},
        {"detector", detector.toJson()},
        {"stereo",   stereo.toJson()},
    };
}

AppConfig AppConfig::loadFromFile(const std::string& path, bool* loaded) {
    AppConfig cfg;
    if (loaded) *loaded = false;

    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        spdlog::info("[AppConfig] No config file at '{}'; using compiled defaults.", path);
        return cfg;
    }

    std::ifstream in(path);
    if (!in.is_open()) {
        spdlog::error("[AppConfig] Could not open '{}'; using compiled defaults.", path);
        return cfg;
    }

    try {
        nlohmann::json j;
        in >> j;
        cfg = AppConfig::fromJson(j, cfg);
        cfg.sourcePath = path;
        if (loaded) *loaded = true;
        spdlog::info("[AppConfig] Loaded '{}'", path);
    } catch (const std::exception& e) {
        spdlog::error("[AppConfig] Failed to parse '{}': {}. Using compiled defaults.", path, e.what());
        cfg = AppConfig();
    }
    return cfg;
}

std::string AppConfig::describe() const {
    std::ostringstream os;
    os << "exposure " << camera.exposureUs << " us (hardware step "
       << CameraConfig::quantiseExposureUs(camera.exposureUs) << " us), gain " << camera.gain
       << ", brightness " << camera.brightness << ", target " << camera.targetFps << " fps"
       << " | intensity threshold " << detector.intensityThreshold
       << ", cluster radius " << detector.clusterRadiusPx << " px, min dots " << detector.minDotsPerCluster
       << " | epipolar tol " << stereo.epipolarTolerancePx << " px, disparity "
       << stereo.disparityMinPx << ".." << stereo.disparityMaxPx << " px"
       << (stereo.swapCameras ? ", cameras swapped" : "")
       << " | source: " << (sourcePath.empty() ? "compiled defaults" : sourcePath);
    return os.str();
}
