#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>
class IUsbVideoDriver {
public:
    virtual ~IUsbVideoDriver() = default;

    /// @brief Capture a raw frame into a pre-allocated destination buffer.
    /// @param destination Pre-allocated cv::Mat (correct size/type). Data is memcpy'd in.
    /// @return true if frame was captured successfully.
    virtual bool grabRawFrame(cv::Mat& destination) = 0;

    virtual void setHardwareExposure(int microseconds) = 0;

    // CRITICAL: Bypasses video stream queue to send direct USB Control Transfers (XU commands).
    // Allows microsecond I2C register updates.
    virtual void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) = 0;
};
