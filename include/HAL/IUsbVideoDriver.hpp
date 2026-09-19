#pragma once
#include "Camera/CameraConfig.hpp"
#include <opencv2/core/mat.hpp>
#include <cstdint>

class IUsbVideoDriver {
public:
    virtual ~IUsbVideoDriver() = default;

    /// @brief Capture a raw frame into a pre-allocated destination buffer.
    /// @param destination Pre-allocated cv::Mat (correct size/type). Data is memcpy'd in.
    /// @return true if frame was captured successfully.
    virtual bool grabRawFrame(cv::Mat& destination) = 0;

    // --- Camera controls: mode transitions only, never on the hot path ---
    // UVC exposure is quantised to log2 steps; the applied value is read back
    // with getHardwareExposureUs(). Gain and brightness are 0..100 / 0..64.
    virtual void setHardwareExposure(int microseconds) = 0;
    virtual void setHardwareGain(int /*gain*/) {}
    virtual void setHardwareBrightness(int /*level*/) {}
    virtual void setAutoExposure(bool /*enabled*/) {}
    virtual void setAutoGain(bool /*enabled*/) {}

    /// @brief Exposure the hardware is actually using, in microseconds. -1 if unknown.
    virtual int getHardwareExposureUs() const { return -1; }
    /// @brief Gain the hardware is actually using. -1 if unknown.
    virtual int getHardwareGain() const { return -1; }
    /// @brief Frame rate negotiated with the device at initialisation. 0 if unknown.
    virtual double getNegotiatedFps() const { return 0.0; }

    /// @brief Apply a full CameraConfig in the order UVC needs: auto modes off
    /// first, then exposure, gain, brightness. Backends override the individual
    /// setters; this sequencing lives in one place.
    void applyCameraConfig(const CameraConfig& cfg) {
        setAutoExposure(cfg.autoExposure);
        setAutoGain(cfg.autoGain);
        setHardwareExposure(cfg.exposureUs);
        setHardwareGain(cfg.gain);
        setHardwareBrightness(cfg.brightness);
    }

    // CRITICAL: Bypasses video stream queue to send direct USB Control Transfers (XU commands).
    // Allows microsecond I2C register updates.
    virtual void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) = 0;

    /// @brief Capture timestamp of the most recent grabRawFrame(), in microseconds
    /// on a monotonic clock. Returns 0 if the backend does not provide one.
    virtual uint64_t getLastFrameTimestampUs() const { return 0; }

    virtual void shutdown() {}
};
