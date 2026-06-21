#pragma once
#include "HAL/IUsbVideoDriver.hpp"
#ifdef _WIN32
class MediaFoundationDriver : public IUsbVideoDriver {
private:
    void* source; // IMFMediaSource*
public:
    cv::Mat grabRawFrame() override;
    void setHardwareExposure(int microseconds) override;
    void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) override;
};
#endif
