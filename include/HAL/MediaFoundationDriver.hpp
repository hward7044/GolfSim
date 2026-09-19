#pragma once
#include "HAL/IUsbVideoDriver.hpp"
#include "Camera/CameraConfig.hpp"
#ifdef _WIN32

// Windows Media Foundation & Kernel Streaming headers
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <ks.h>
#include <ksproxy.h>
#include <strmif.h>       // IAMCameraControl, IAMVideoProcAmp
#include <wrl/client.h>   // Microsoft::WRL::ComPtr
#include <string>
#include <vector>
#include <cstdint>

using Microsoft::WRL::ComPtr;

class MediaFoundationDriver : public IUsbVideoDriver {
public:
    /// One native media type advertised by the device, as logged at startup.
    struct MediaTypeInfo {
        DWORD    index   = 0;
        GUID     subtype = GUID_NULL;
        uint32_t width   = 0;
        uint32_t height  = 0;
        double   fps     = 0.0;
        bool     usable  = false;   // L8 or NV12: a Y plane we can copy without decoding
    };

private:
    uint32_t                 deviceIndex_ = 0;
    CameraConfig             config_;

    // --- Lifecycle guard ---
    bool initialized_ = false;
    bool comInitializedByUs_ = false;
    bool mfInitializedByUs_  = false;

    // --- Media Foundation pipeline ---
    ComPtr<IMFMediaSource>   mediaSource_;
    ComPtr<IMFSourceReader>  sourceReader_;

    // --- UVC control interfaces, cached at init (off the hot path, but no
    //     reason to QueryInterface on every setter call) ---
    ComPtr<IAMCameraControl> cameraControl_;
    ComPtr<IAMVideoProcAmp>  procAmp_;
    long exposureLog2Min_ = CameraConfig::kMinExposureLog2;
    long exposureLog2Max_ = CameraConfig::kMaxExposureLog2;
    long gainMin_ = 0, gainMax_ = CameraConfig::kMaxGain;
    long brightnessMin_ = 0, brightnessMax_ = CameraConfig::kMaxBrightness;
    int    appliedExposureUs_ = -1;
    int    appliedGain_       = -1;
    double negotiatedFps_     = 0.0;

    // --- Extension Unit (XU) for I2C passthrough ---
    ComPtr<IKsControl>       ksControl_;
    DWORD                    xuNodeId_ = 0;

    // --- Cached for hot-path performance: pre-built KSPROPERTY template ---
    KSPROPERTY               cachedKspProp_{};

    // --- Frame geometry (resolved at init) ---
    uint32_t frameWidth_  = 0;
    uint32_t frameHeight_ = 0;
    int32_t  frameStride_ = 0;   // Signed: MF can return negative stride for bottom-up
    bool     isNV12_      = false;
    uint64_t lastTimestampUs_ = 0;  // From IMFSourceReader::ReadSample (100 ns -> us)

    // --- Internal helpers ---
    bool initializeMediaFoundation();
    bool enumerateAndOpenDevice();
    bool configureSourceReader();
    bool discoverExtensionUnit();
    void cacheControlInterfaces();
    void applyStartupConfig();
    std::vector<MediaTypeInfo> enumerateNativeMediaTypes() const;
    static const MediaTypeInfo* selectMediaType(const std::vector<MediaTypeInfo>& types,
                                                uint32_t width, uint32_t height, int targetFps);

public:
    explicit MediaFoundationDriver(uint32_t deviceIndex = 0, CameraConfig config = CameraConfig());
    ~MediaFoundationDriver();

    // Prevent copy and move
    MediaFoundationDriver(const MediaFoundationDriver&) = delete;
    MediaFoundationDriver& operator=(const MediaFoundationDriver&) = delete;
    MediaFoundationDriver(MediaFoundationDriver&&) = delete;
    MediaFoundationDriver& operator=(MediaFoundationDriver&&) = delete;

    /// @brief Initialize the full MF pipeline. Must be called before grabRawFrame().
    bool initialize();

    /// @brief Release all COM resources and shut down MF. Safe to call multiple times.
    void shutdown() override;

    /// @brief Check if the driver has been successfully initialized.
    bool isInitialized() const noexcept { return initialized_; }

    /// @brief Returns the frame width resolved at initialization.
    uint32_t getFrameWidth() const noexcept { return frameWidth_; }
    /// @brief Returns the frame height resolved at initialization.
    uint32_t getFrameHeight() const noexcept { return frameHeight_; }

    /// @brief The configuration this driver was constructed with.
    const CameraConfig& getConfig() const noexcept { return config_; }

    /// @brief Enumerate and log all connected video capture devices.
    static void logConnectedDevices();

    // --- IUsbVideoDriver interface ---
    bool grabRawFrame(cv::Mat& destination) override;
    void setHardwareExposure(int microseconds) override;
    void setHardwareGain(int gain) override;
    void setHardwareBrightness(int level) override;
    void setAutoExposure(bool enabled) override;
    void setAutoGain(bool enabled) override;
    int  getHardwareExposureUs() const override { return appliedExposureUs_; }
    int  getHardwareGain() const override { return appliedGain_; }
    double getNegotiatedFps() const override { return negotiatedFps_; }
    void injectImmediateRegisterWrite(uint16_t reg, uint8_t value) override;
    uint64_t getLastFrameTimestampUs() const override { return lastTimestampUs_; }
};
#endif
