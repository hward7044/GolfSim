#pragma once
#include "Camera/ICameraNode.hpp"
#include "HAL/IUsbVideoDriver.hpp"
#include <memory>
class OV9281CameraNode : public ICameraNode {
private:
    std::unique_ptr<IUsbVideoDriver> usbDriver;
    // Triggers driver->injectImmediateRegisterWrite() to pull physical STROBE pin high on chip.
    void enableHardwareStrobeMode();
public:
    cv::Mat captureFrame() override;
    CameraRole getRole() override;
};
