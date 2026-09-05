#include "Camera/OV9281CameraNode.hpp"

OV9281CameraNode::OV9281CameraNode(std::unique_ptr<IUsbVideoDriver> driver, CameraRole role)
    : usbDriver(std::move(driver)), role_(role) {}

bool OV9281CameraNode::captureFrame(cv::Mat& destination) {
    if (!usbDriver) return false;
    return usbDriver->grabRawFrame(destination);
}

CameraRole OV9281CameraNode::getRole() {
    return role_;
}

void OV9281CameraNode::setExposure(int microseconds) {
    if (usbDriver) {
        usbDriver->setHardwareExposure(microseconds);
    }
}

void OV9281CameraNode::shutdown() {
    if (usbDriver) {
        usbDriver->shutdown();
    }
}
