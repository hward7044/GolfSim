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

void OV9281CameraNode::applyConfig(const CameraConfig& config) {
    if (usbDriver) {
        usbDriver->applyCameraConfig(config);
    }
}

void OV9281CameraNode::setExposure(int microseconds) {
    if (usbDriver) {
        usbDriver->setHardwareExposure(microseconds);
    }
}

void OV9281CameraNode::setGain(int gain) {
    if (usbDriver) {
        usbDriver->setHardwareGain(gain);
    }
}

int OV9281CameraNode::getAppliedExposureUs() const {
    return usbDriver ? usbDriver->getHardwareExposureUs() : -1;
}

int OV9281CameraNode::getAppliedGain() const {
    return usbDriver ? usbDriver->getHardwareGain() : -1;
}

double OV9281CameraNode::getNegotiatedFps() const {
    return usbDriver ? usbDriver->getNegotiatedFps() : 0.0;
}

uint64_t OV9281CameraNode::getLastFrameTimestampUs() const {
    return usbDriver ? usbDriver->getLastFrameTimestampUs() : 0;
}

void OV9281CameraNode::shutdown() {
    if (usbDriver) {
        usbDriver->shutdown();
    }
}
