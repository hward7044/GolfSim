#pragma once
#include "HAL/IUsbVideoDriver.hpp"
#ifdef __linux__
class V4L2Driver : public IUsbVideoDriver {
private:
    int fileDescriptor;
public:
    cv::Mat grabRawFrame() override;
    void setHardwareExposure(int microseconds) override;
    void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) override;
};
#endif
