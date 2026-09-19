#include "Camera/CameraConfig.hpp"
#include <algorithm>
#include <cmath>

CameraConfig CameraConfig::fromJson(const nlohmann::json& j, const CameraConfig& base) {
    CameraConfig cfg = base;
    if (!j.is_object()) return cfg;
    cfg.exposureUs   = j.value("exposureUs",   cfg.exposureUs);
    cfg.gain         = j.value("gain",         cfg.gain);
    cfg.brightness   = j.value("brightness",   cfg.brightness);
    cfg.targetFps    = j.value("targetFps",    cfg.targetFps);
    cfg.autoExposure = j.value("autoExposure", cfg.autoExposure);
    cfg.autoGain     = j.value("autoGain",     cfg.autoGain);
    cfg.clampToHardwareRanges();
    return cfg;
}

nlohmann::json CameraConfig::toJson() const {
    return {
        {"exposureUs",   exposureUs},
        {"gain",         gain},
        {"brightness",   brightness},
        {"targetFps",    targetFps},
        {"autoExposure", autoExposure},
        {"autoGain",     autoGain},
    };
}

bool CameraConfig::clampToHardwareRanges() {
    const CameraConfig before = *this;
    gain       = std::clamp(gain, 0, kMaxGain);
    brightness = std::clamp(brightness, 0, kMaxBrightness);
    if (exposureUs < 0) exposureUs = 0;
    if (targetFps < 1)  targetFps = 1;
    return gain != before.gain || brightness != before.brightness ||
           exposureUs != before.exposureUs || targetFps != before.targetFps;
}

int CameraConfig::exposureUsToLog2(int microseconds) noexcept {
    if (microseconds <= 0) return kMinExposureLog2;
    // value = log2(seconds); round to the nearest step so 3000 us -> 3906 us,
    // not the 1953 us that floor() would silently give.
    const double log2Seconds = std::log2(static_cast<double>(microseconds) * 1e-6);
    const int value = static_cast<int>(std::lround(log2Seconds));
    return std::clamp(value, kMinExposureLog2, kMaxExposureLog2);
}

int CameraConfig::exposureLog2ToUs(int log2Value) noexcept {
    // Truncate: 2^-7 s is exactly 7812.5 us and the value quoted everywhere is 7812.
    return static_cast<int>(std::ldexp(1.0, log2Value) * 1e6);
}
