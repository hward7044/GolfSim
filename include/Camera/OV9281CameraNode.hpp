#pragma once
#include "Camera/ICameraNode.hpp"
#include "HAL/IUsbVideoDriver.hpp"
#include "Camera/CameraConfig.hpp"
#include <memory>

class OV9281CameraNode : public ICameraNode {
private:
    std::unique_ptr<IUsbVideoDriver> usbDriver;
    CameraRole role_;
public:
    explicit OV9281CameraNode(std::unique_ptr<IUsbVideoDriver> driver, CameraRole role);
    bool captureFrame(cv::Mat& destination) override;
    CameraRole getRole() override;
    // Camera controls (mode transitions only, never on the hot path)
    void applyConfig(const CameraConfig& config);
    void setExposure(int microseconds);
    void setGain(int gain);
    /// Exposure the hardware is actually using after UVC quantisation, in us (-1 if unknown).
    int  getAppliedExposureUs() const;
    int  getAppliedGain() const;
    double getNegotiatedFps() const;
    uint64_t getLastFrameTimestampUs() const override;
    void shutdown() override;
};
