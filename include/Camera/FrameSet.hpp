#pragma once
#include <opencv2/core/mat.hpp>
#include <array>
#include <cstdint>
#include "Camera/CameraRole.hpp"

class FrameSet {
public:
    uint64_t timestamp = 0;

    /// Pre-allocated frame storage — sized automatically from CameraRole::COUNT.
    /// Adding a new camera role to the enum is all that's needed; no manual edits here.
    std::array<cv::Mat, CAMERA_ROLE_COUNT> frames;

    cv::Mat& getFrame(CameraRole role) {
        return frames[static_cast<std::size_t>(role)];
    }

    const cv::Mat& getFrame(CameraRole role) const {
        return frames[static_cast<std::size_t>(role)];
    }

    /// Pre-allocate the cv::Mat buffers at the given dimensions (called once at boot).
    void preallocate(uint32_t width, uint32_t height) {
        for (auto& f : frames) {
            f = cv::Mat(height, width, CV_8UC1);
        }
    }

    /// Zero-copy fast pointer swap between two FrameSets
    void swap(FrameSet& other) noexcept {
        std::swap(timestamp, other.timestamp);
        for (std::size_t i = 0; i < CAMERA_ROLE_COUNT; ++i) {
            std::swap(frames[i], other.frames[i]);
        }
    }

    friend void swap(FrameSet& a, FrameSet& b) noexcept {
        a.swap(b);
    }

    /// Deep copy matrix pixels into pre-allocated destination without reallocation
    void copyTo(FrameSet& dest) const {
        dest.timestamp = timestamp;
        for (std::size_t i = 0; i < CAMERA_ROLE_COUNT; ++i) {
            if (!frames[i].empty()) {
                frames[i].copyTo(dest.frames[i]);
            }
        }
    }
};
