#pragma once
#include "Camera/ICameraNode.hpp"
#include "HAL/IUsbVideoDriver.hpp"
#include <memory>
class OV9281CameraNode : public ICameraNode {
private:
    std::unique_ptr<IUsbVideoDriver> usbDriver;
public:
    cv::Mat captureFrame() override;
    CameraRole getRole() override;
};
