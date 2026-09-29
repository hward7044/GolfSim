#include "App/AppConfig.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <spdlog/spdlog.h>
#ifdef _WIN32
#include <windows.h>
#endif

namespace {

// Directory holding the running executable; empty if the OS will not say.
std::filesystem::path executableDirectory() {
#ifdef _WIN32
    wchar_t buf[MAX_PATH];
    DWORD n = GetModuleFileNameW(nullptr, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    return std::filesystem::path(buf).parent_path();
#else
    std::error_code ec;
    std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (ec) return {};
    return exe.parent_path();
#endif
}

} // namespace

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
// StrobeConfig
// =============================================================================

bool StrobeConfig::clampToEnvelope() {
    const int before = pulseWidthUs;
    // Both limits bind: the absolute width for blur, the duty for average power.
    const int dutyLimitUs = static_cast<int>(kMaxDutyCycle * 1e6 / kRateHz);
    pulseWidthUs = std::clamp(pulseWidthUs, kMinPulseWidthUs, std::min(kMaxPulseWidthUs, dutyLimitUs));
    return pulseWidthUs != before;
}

StrobeConfig StrobeConfig::fromJson(const nlohmann::json& j, const StrobeConfig& base) {
    StrobeConfig cfg = base;
    if (!j.is_object()) return cfg;
    cfg.pulseWidthUs = j.value("pulseWidthUs", cfg.pulseWidthUs);
    if (cfg.clampToEnvelope()) {
        spdlog::warn("[StrobeConfig] pulseWidthUs clamped to {} us (envelope {}..{} us, duty <= {:.0f}% at {} Hz)",
                     cfg.pulseWidthUs, kMinPulseWidthUs, kMaxPulseWidthUs, kMaxDutyCycle * 100.0, kRateHz);
    }
    return cfg;
}

nlohmann::json StrobeConfig::toJson() const {
    return {
        {"pulseWidthUs", pulseWidthUs},
    };
}

// =============================================================================
// AppConfig
// =============================================================================

std::string AppConfig::findProjectRootAbove(const std::filesystem::path& start) {
    std::error_code ec;
    std::filesystem::path dir = std::filesystem::absolute(start, ec);
    if (ec) return {};
    dir = dir.lexically_normal();
    if (dir.filename().empty()) dir = dir.parent_path();  // drop a trailing separator

    while (!dir.empty()) {
        if (std::filesystem::is_regular_file(dir / kDefaultPath, ec)) return dir.string();
        std::filesystem::path parent = dir.parent_path();
        if (parent == dir) break;
        dir = parent;
    }
    return {};
}

std::string AppConfig::findProjectRoot() {
    std::error_code ec;
    std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec) {
        std::string root = findProjectRootAbove(cwd);
        if (!root.empty()) return root;
    }
    std::filesystem::path exeDir = executableDirectory();
    if (!exeDir.empty()) return findProjectRootAbove(exeDir);
    return {};
}

AppConfig AppConfig::fromJson(const nlohmann::json& j, const AppConfig& base) {
    AppConfig cfg = base;
    if (!j.is_object()) return cfg;
    if (j.contains("camera"))   cfg.camera   = CameraConfig::fromJson(j["camera"], cfg.camera);
    if (j.contains("detector")) cfg.detector = DotClusterConfig::fromJson(j["detector"], cfg.detector);
    if (j.contains("stereo"))   cfg.stereo   = StereoConfig::fromJson(j["stereo"], cfg.stereo);
    if (j.contains("strobe"))   cfg.strobe   = StrobeConfig::fromJson(j["strobe"], cfg.strobe);
    return cfg;
}

nlohmann::json AppConfig::toJson() const {
    return {
        {"camera",   camera.toJson()},
        {"detector", detector.toJson()},
        {"stereo",   stereo.toJson()},
        {"strobe",   strobe.toJson()},
    };
}

AppConfig AppConfig::loadFromFile(const std::string& path, bool* loaded) {
    AppConfig cfg;
    if (loaded) *loaded = false;

    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) {
        std::filesystem::path tried = std::filesystem::absolute(path, ec);
        spdlog::warn("[AppConfig] No config file at '{}'; using compiled defaults.",
                     ec ? path : tried.string());
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
       << " | strobe " << strobe.pulseWidthUs << " us at " << StrobeConfig::kRateHz << " Hz ("
       << std::fixed << std::setprecision(1) << strobe.dutyCycle() * 100.0 << "% duty)"
       << " | source: " << (sourcePath.empty() ? "compiled defaults" : sourcePath);
    return os.str();
}
