#pragma once
#include <opencv2/opencv.hpp>
class IUsbVideoDriver {
public:
    virtual ~IUsbVideoDriver() = default;
    virtual cv::Mat grabRawFrame() = 0;
    virtual void setHardwareExposure(int microseconds) = 0;
};
