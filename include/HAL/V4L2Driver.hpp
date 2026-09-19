#pragma once
#include "HAL/IUsbVideoDriver.hpp"
#ifdef __linux__

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

/// Linux Video4Linux2 driver for the OV9281 UVC cameras.
///
/// Mirrors MediaFoundationDriver's public surface so main.cpp can select the
/// platform driver with a single alias. Uses raw V4L2 ioctls with MMAP
/// streaming: explicit pixel-format negotiation (GREY -> NV12 -> YUYV),
/// a shallow 4-buffer queue, manual UVC exposure, kernel frame timestamps,
/// and a poll()-based grab that shutdown() can interrupt from another thread.
class V4L2Driver : public IUsbVideoDriver {
private:
    struct MmapBuffer {
        void*  start  = nullptr;
        size_t length = 0;
    };

    std::string devicePath_;
    int         fd_ = -1;

    // --- Lifecycle guard ---
    bool              initialized_ = false;
    std::atomic<bool> streaming_{false};
    std::mutex        grabMutex_;   // Serialises grabRawFrame() against shutdown()

    // --- Frame geometry (resolved at init) ---
    uint32_t width_  = 0;
    uint32_t height_ = 0;
    uint32_t stride_ = 0;   // bytesperline reported by the driver
    uint32_t pixFmt_ = 0;   // V4L2_PIX_FMT_* actually negotiated

    // --- Exposure control range (100 us units, resolved at init) ---
    int32_t exposureMin_ = 1;
    int32_t exposureMax_ = 5000;
    bool    exposureSupported_ = false;

    std::vector<MmapBuffer> buffers_;
    uint64_t lastTimestampUs_ = 0;

    // --- Internal helpers ---
    bool openDevice();
    bool negotiateFormat();
    bool selectMaxFrameRate();
    void configureControls();
    bool setupBuffers();
    bool startStreaming();
    void releaseBuffers();

public:
    /// @param logicalIndex Nth capture-capable /dev/video* node (metadata nodes are skipped).
    explicit V4L2Driver(uint32_t logicalIndex = 0);
    /// @param devicePath Explicit node, e.g. "/dev/video2" or "/dev/v4l/by-id/usb-...-video-index0".
    explicit V4L2Driver(std::string devicePath);
    ~V4L2Driver();

    // Prevent copy and move
    V4L2Driver(const V4L2Driver&) = delete;
    V4L2Driver& operator=(const V4L2Driver&) = delete;
    V4L2Driver(V4L2Driver&&) = delete;
    V4L2Driver& operator=(V4L2Driver&&) = delete;

    /// @brief Open, negotiate format/rate, apply manual exposure, map buffers and start streaming.
    bool initialize();

    /// @brief Stop streaming, unmap buffers and close the device. Safe to call multiple times.
    void shutdown() override;

    bool isInitialized() const noexcept { return initialized_; }
    uint32_t getFrameWidth() const noexcept { return width_; }
    uint32_t getFrameHeight() const noexcept { return height_; }
    const std::string& getDevicePath() const noexcept { return devicePath_; }

    /// @brief Enumerate and log all capture-capable V4L2 devices.
    static void logConnectedDevices();

    /// @brief Sorted list of /dev/video* nodes that expose a video capture format.
    static std::vector<std::string> enumerateCaptureDevices();

    // --- IUsbVideoDriver interface ---
    bool grabRawFrame(cv::Mat& destination) override;
    void setHardwareExposure(int microseconds) override;
    void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) override;
    uint64_t getLastFrameTimestampUs() const override { return lastTimestampUs_; }
};

#endif
