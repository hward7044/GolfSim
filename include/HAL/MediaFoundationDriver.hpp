#pragma once
#include "HAL/IUsbVideoDriver.hpp"
#ifdef _WIN32

// Windows Media Foundation & Kernel Streaming headers
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <ks.h>
#include <ksproxy.h>
#include <wrl/client.h>   // Microsoft::WRL::ComPtr
#include <string>
#include <cstdint>

using Microsoft::WRL::ComPtr;

class MediaFoundationDriver : public IUsbVideoDriver {
private:
    // --- Lifecycle guard ---
    bool initialized_ = false;
    bool comInitializedByUs_ = false;
    bool mfInitializedByUs_  = false;

    // --- Media Foundation pipeline ---
    ComPtr<IMFMediaSource>   mediaSource_;
    ComPtr<IMFSourceReader>  sourceReader_;

    // --- Extension Unit (XU) for I2C passthrough ---
    ComPtr<IKsControl>       ksControl_;
    DWORD                    xuNodeId_ = 0;

    // --- Cached for hot-path performance: pre-built KSP_NODE template ---
    KSP_NODE                 cachedKspNode_{};

    // --- Frame geometry (resolved at init) ---
    uint32_t frameWidth_  = 0;
    uint32_t frameHeight_ = 0;
    int32_t  frameStride_ = 0;   // Signed: MF can return negative stride for bottom-up
    bool     isNV12_      = false;

    // --- Internal helpers ---
    bool initializeMediaFoundation();
    bool enumerateAndOpenDevice();
    bool configureSourceReader();
    bool discoverExtensionUnit();

public:
    MediaFoundationDriver();
    ~MediaFoundationDriver();

    // Prevent copy and move
    MediaFoundationDriver(const MediaFoundationDriver&) = delete;
    MediaFoundationDriver& operator=(const MediaFoundationDriver&) = delete;
    MediaFoundationDriver(MediaFoundationDriver&&) = delete;
    MediaFoundationDriver& operator=(MediaFoundationDriver&&) = delete;

    /// @brief Initialize the full MF pipeline. Must be called before grabRawFrame().
    bool initialize();

    /// @brief Release all COM resources and shut down MF. Safe to call multiple times.
    void shutdown();

    /// @brief Check if the driver has been successfully initialized.
    bool isInitialized() const noexcept { return initialized_; }

    /// @brief Returns the frame width resolved at initialization.
    uint32_t getFrameWidth() const noexcept { return frameWidth_; }
    /// @brief Returns the frame height resolved at initialization.
    uint32_t getFrameHeight() const noexcept { return frameHeight_; }

    // --- IUsbVideoDriver interface ---
    bool grabRawFrame(cv::Mat& destination) override;
    void setHardwareExposure(int microseconds) override;
    void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) override;
};
#endif
