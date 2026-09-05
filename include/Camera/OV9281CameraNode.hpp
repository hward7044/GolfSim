#pragma once
#include "Camera/ICameraNode.hpp"
#include "HAL/IUsbVideoDriver.hpp"
#include <memory>

class OV9281CameraNode : public ICameraNode {
private:
    std::unique_ptr<IUsbVideoDriver> usbDriver;
    CameraRole role_;
public:
    explicit OV9281CameraNode(std::unique_ptr<IUsbVideoDriver> driver, CameraRole role);
    bool captureFrame(cv::Mat& destination) override;
    CameraRole getRole() override;
    void enableHardwareStrobeMode();
    void setStrobe(bool enable);
    void setExposure(int microseconds);
    void shutdown() override;
};
