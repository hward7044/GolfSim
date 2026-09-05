#include "Camera/OV9281CameraNode.hpp"
#include "HAL/OV9281Registers.hpp"

OV9281CameraNode::OV9281CameraNode(std::unique_ptr<IUsbVideoDriver> driver, CameraRole role)
    : usbDriver(std::move(driver)), role_(role) {}

bool OV9281CameraNode::captureFrame(cv::Mat& destination) {
    if (!usbDriver) return false;
    return usbDriver->grabRawFrame(destination);
}

CameraRole OV9281CameraNode::getRole() {
    return role_;
}

#include "HAL/MediaFoundationDriver.hpp"

void OV9281CameraNode::enableHardwareStrobeMode() {
    setStrobe(true);
}

void OV9281CameraNode::setStrobe(bool enable) {
    // Disabled: Hardware strobe configuration via UVC is unsupported on this bridge.
    // The Arduino MCU acts as the trigger master via Serial commands instead.
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
