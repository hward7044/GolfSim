#pragma once
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <cstdint>
#include "CameraRole.hpp"
class FrameSet {
public:
    uint64_t timestamp;
private:
    std::unordered_map<CameraRole, cv::Mat> frames;
public:
    cv::Mat getFrame(CameraRole role);
};
