#pragma once
#include "ICameraNode.hpp"
#include "IUsbVideoDriver.hpp"
#include <memory>
class OV9281CameraNode : public ICameraNode {
private:
    std::unique_ptr<IUsbVideoDriver> usbDriver;
public:
    cv::Mat captureFrame() override;
    CameraRole getRole() override;
};
