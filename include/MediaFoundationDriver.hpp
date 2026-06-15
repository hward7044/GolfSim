#pragma once
#include "IUsbVideoDriver.hpp"
#ifdef _WIN32
class MediaFoundationDriver : public IUsbVideoDriver {
private:
    void* source; // IMFMediaSource*
public:
    cv::Mat grabRawFrame() override;
    void setHardwareExposure(int microseconds) override;
};
#endif
