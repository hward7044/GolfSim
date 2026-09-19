#include "HAL/MediaFoundationDriver.hpp"
#ifdef _WIN32

#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <utility>
#include <guiddef.h>
#include <devpkey.h>
#include <strmif.h>       // IAMCameraControl
#include <vidcap.h>       // IKsTopologyInfo
#include <ksmedia.h>      // KSNODETYPE_DEV_SPECIFIC
#include <spdlog/spdlog.h>
#include "HAL/OV9281Registers.hpp"

// Link directives are handled in CMake, but these pragmas serve as documentation
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

// =============================================================================
// Device identification constants
// =============================================================================
// Arducam/OmniVision OV9281 default VID/PID
// TODO: Verify these against your specific hardware via Device Manager → Hardware IDs
static const std::wstring TARGET_VID = L"VID_0C45";
static const std::wstring TARGET_PID = L"PID_6366";

// PLACEHOLDER XU GUID: Replace with the actual GUID from your camera's UVC Extension Unit descriptor.
// Discoverable via USBView.exe, Wireshark+USBPcap, or the Arducam SDK documentation.
// {28105740-4C3D-4B44-AA94-B6E911F24781} is a common Arducam controller XU GUID.
static const GUID ARDUCAM_XU_GUID = {
    0x28105740, 0x4C3D, 0x4B44,
    { 0xAA, 0x94, 0xB6, 0xE9, 0x11, 0xF2, 0x47, 0x81 }
};
static constexpr DWORD ARDUCAM_XU_CONTROL_ID = 1;

// FrameSet::preallocate(1280, 800) is hardcoded upstream; the ring buffer's
// zero-allocation contract depends on the camera delivering exactly this.
static constexpr uint32_t REQUESTED_WIDTH  = 1280;
static constexpr uint32_t REQUESTED_HEIGHT = 800;

static const char* subtypeName(const GUID& subtype) {
    if (subtype == MFVideoFormat_L8)    return "L8";
    if (subtype == MFVideoFormat_NV12)  return "NV12";
    if (subtype == MFVideoFormat_YUY2)  return "YUY2";
    if (subtype == MFVideoFormat_MJPG)  return "MJPG";
    if (subtype == MFVideoFormat_RGB24) return "RGB24";
    if (subtype == MFVideoFormat_L16)   return "L16";
    return "other";
}

// =============================================================================
// Construction / Destruction
// =============================================================================

MediaFoundationDriver::MediaFoundationDriver(uint32_t deviceIndex, CameraConfig config)
    : deviceIndex_(deviceIndex), config_(std::move(config)) {
    config_.clampToHardwareRanges();
}

MediaFoundationDriver::~MediaFoundationDriver() {
    shutdown();
}

// =============================================================================
// Lifecycle: initialize / shutdown
// =============================================================================

bool MediaFoundationDriver::initialize() {
    if (initialized_) return true;  // Idempotent

    if (!initializeMediaFoundation()) {
        spdlog::error("[MediaFoundationDriver] Failed to initialize COM/MF");
        shutdown();
        return false;
    }
    if (!enumerateAndOpenDevice()) {
        spdlog::error("[MediaFoundationDriver] Target OV9281 camera not found/opened.");
        shutdown();
        return false;
    }
    if (!configureSourceReader()) {
        spdlog::error("[MediaFoundationDriver] Failed to configure source reader media types.");
        shutdown();
        return false;
    }

    // Attempt to locate and configure the USB Extension Unit for I2C writes
    if (!discoverExtensionUnit()) {
        spdlog::warn("[MediaFoundationDriver] Extension Unit discovery failed. Register writes will be unavailable.");
        // Non-fatal: camera can still grab frames
    } else {
        // Pre-build the KSP_NODE template so injectImmediateRegisterWrite()
        // has zero setup overhead on the hot path
        ZeroMemory(&cachedKspProp_, sizeof(cachedKspProp_));
        cachedKspProp_.Set   = ARDUCAM_XU_GUID;
        cachedKspProp_.Id    = ARDUCAM_XU_CONTROL_ID;
        cachedKspProp_.Flags = KSPROPERTY_TYPE_SET;
    }

    initialized_ = true;

    // UVC controls: cache the interfaces once, then push the configured
    // exposure / gain / brightness (failures logged as warnings, never fatal)
    cacheControlInterfaces();
    applyStartupConfig();

    return true;
}

void MediaFoundationDriver::shutdown() {
    // Guard against double-shutdown (destructor + explicit call)
    initialized_ = false;

    // First shut down the media source to unblock any pending ReadSample() calls
    if (mediaSource_) {
        mediaSource_->Shutdown();
    }

    // Release in reverse order of acquisition
    ksControl_.Reset();
    procAmp_.Reset();
    cameraControl_.Reset();
    sourceReader_.Reset();
    mediaSource_.Reset();

    // Only tear down subsystems we initialized
    if (mfInitializedByUs_) {
        MFShutdown();
        mfInitializedByUs_ = false;
    }
    if (comInitializedByUs_) {
        CoUninitialize();
        comInitializedByUs_ = false;
    }
}

// =============================================================================
// Internal: COM & Media Foundation bootstrap
// =============================================================================

bool MediaFoundationDriver::initializeMediaFoundation() {
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    if (hr == RPC_E_CHANGED_MODE) {
        // COM already initialized in a different mode on this thread — that's acceptable.
        comInitializedByUs_ = false;
    } else if (FAILED(hr)) {
        return false;
    } else {
        comInitializedByUs_ = true;
    }

    hr = MFStartup(MF_VERSION);
    if (FAILED(hr)) {
        if (comInitializedByUs_) {
            CoUninitialize();
            comInitializedByUs_ = false;
        }
        return false;
    }
    mfInitializedByUs_ = true;
    return true;
}

// =============================================================================
// Internal: Device enumeration — filter by VID/PID
// =============================================================================

bool MediaFoundationDriver::enumerateAndOpenDevice() {
    ComPtr<IMFAttributes> pAttributes;
    HRESULT hr = MFCreateAttributes(&pAttributes, 1);
    if (FAILED(hr)) return false;

    hr = pAttributes->SetGUID(
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
    );
    if (FAILED(hr)) return false;

    UINT32 count = 0;
    IMFActivate** ppDevices = nullptr;
    hr = MFEnumDeviceSources(pAttributes.Get(), &ppDevices, &count);
    if (FAILED(hr) || count == 0) return false;

    bool found = false;
    uint32_t matchCount = 0;
    for (UINT32 i = 0; i < count; ++i) {
        if (!ppDevices[i]) continue;  // Defensive null check

        if (found) {
            ppDevices[i]->Release();
            continue;
        }

        WCHAR* szFriendlyName = nullptr;
        UINT32 cchFriendlyName = 0;
        ppDevices[i]->GetAllocatedString(
            MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &szFriendlyName, &cchFriendlyName
        );

        WCHAR* szLinks = nullptr;
        UINT32 cchLinks = 0;
        ppDevices[i]->GetAllocatedString(
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &szLinks, &cchLinks
        );

        std::wstring linkStr(szLinks ? szLinks : L"");
        std::wstring nameStr(szFriendlyName ? szFriendlyName : L"");

        CoTaskMemFree(szFriendlyName);
        CoTaskMemFree(szLinks);

        bool isMatch = false;
        if (linkStr.find(TARGET_VID) != std::wstring::npos &&
            linkStr.find(TARGET_PID) != std::wstring::npos) {
            isMatch = true;
        } else if (nameStr.find(L"Arducam") != std::wstring::npos ||
                   nameStr.find(L"OV9281")  != std::wstring::npos) {
            isMatch = true;
        }

        if (isMatch) {
            if (matchCount == deviceIndex_) {
                hr = ppDevices[i]->ActivateObject(IID_PPV_ARGS(&mediaSource_));
                if (SUCCEEDED(hr)) {
                    found = true;
                    std::string narrowName(nameStr.begin(), nameStr.end());
                    spdlog::info("[MediaFoundationDriver] Successfully activated device at index {} (Name: {})",
                                 deviceIndex_, narrowName);
                }
            }
            matchCount++;
        }

        ppDevices[i]->Release();
    }
    CoTaskMemFree(ppDevices);
    return found;
}

void MediaFoundationDriver::logConnectedDevices() {
    HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    bool comInit = SUCCEEDED(hr) || hr == RPC_E_CHANGED_MODE;
    bool comShouldUninit = (hr == S_OK || hr == S_FALSE);

    hr = MFStartup(MF_VERSION);
    bool mfInit = SUCCEEDED(hr);

    if (mfInit) {
        ComPtr<IMFAttributes> pAttributes;
        hr = MFCreateAttributes(&pAttributes, 1);
        if (SUCCEEDED(hr)) {
            hr = pAttributes->SetGUID(
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
            );
            if (SUCCEEDED(hr)) {
                UINT32 count = 0;
                IMFActivate** ppDevices = nullptr;
                hr = MFEnumDeviceSources(pAttributes.Get(), &ppDevices, &count);
                if (SUCCEEDED(hr)) {
                    spdlog::info("[MediaFoundationDriver] Enumerating connected video devices (Total: {}):", count);
                    for (UINT32 i = 0; i < count; ++i) {
                        if (!ppDevices[i]) continue;
                        WCHAR* szFriendlyName = nullptr;
                        UINT32 cchFriendlyName = 0;
                        ppDevices[i]->GetAllocatedString(
                            MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &szFriendlyName, &cchFriendlyName
                        );

                        WCHAR* szLinks = nullptr;
                        UINT32 cchLinks = 0;
                        ppDevices[i]->GetAllocatedString(
                            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &szLinks, &cchLinks
                        );

                        std::wstring nameW(szFriendlyName ? szFriendlyName : L"");
                        std::wstring linkW(szLinks ? szLinks : L"");

                        std::string nameStr(nameW.begin(), nameW.end());
                        std::string linkStr(linkW.begin(), linkW.end());

                        spdlog::info("  Device [{}]:", i);
                        spdlog::info("    Name: {}", nameStr);
                        spdlog::info("    Link: {}", linkStr);

                        CoTaskMemFree(szFriendlyName);
                        CoTaskMemFree(szLinks);
                        ppDevices[i]->Release();
                    }
                    CoTaskMemFree(ppDevices);
                }
            }
        }
        MFShutdown();
    }
    if (comInit && comShouldUninit) {
        CoUninitialize();
    }
}

// =============================================================================
// Internal: Source Reader configuration & format negotiation
// =============================================================================

bool MediaFoundationDriver::configureSourceReader() {
    if (!mediaSource_) return false;

    // Configure the source reader for low-latency capture
    ComPtr<IMFAttributes> readerAttrs;
    HRESULT hr = MFCreateAttributes(&readerAttrs, 2);
    if (SUCCEEDED(hr)) {
        // Disable internal sample processing — we want raw frames
        readerAttrs->SetUINT32(MF_SOURCE_READER_DISABLE_DXVA, TRUE);
        // Enable low-latency mode for faster frame delivery
        readerAttrs->SetUINT32(MF_LOW_LATENCY, TRUE);
    }

    hr = MFCreateSourceReaderFromMediaSource(
        mediaSource_.Get(),
        readerAttrs.Get(),  // Pass attributes for low-latency config
        &sourceReader_
    );
    if (FAILED(hr)) return false;

    // -------------------------------------------------------------------------
    // Format negotiation: the device advertises one native type per
    // (subtype, size, frame rate). Log them all, then pick a 1280x800 L8/NV12
    // type that reaches the configured frame rate (refactor 09, section 2.5).
    // -------------------------------------------------------------------------
    const std::vector<MediaTypeInfo> types = enumerateNativeMediaTypes();
    spdlog::info("[MediaFoundationDriver] Device {} advertises {} native media types:",
                 deviceIndex_, types.size());
    for (const auto& t : types) {
        spdlog::info("    [{:2}] {} {}x{} @ {:.1f} fps{}", t.index, subtypeName(t.subtype),
                     t.width, t.height, t.fps, t.usable ? "" : "  (not usable)");
    }

    const MediaTypeInfo* chosen = selectMediaType(types, REQUESTED_WIDTH, REQUESTED_HEIGHT,
                                                  config_.targetFps);
    if (!chosen) {
        std::cerr << "[MediaFoundationDriver] No compatible media type found (L8 or NV12)." << std::endl;
        return false;
    }
    if (chosen->fps + 0.5 < config_.targetFps) {
        spdlog::warn("[MediaFoundationDriver] No {}x{} L8/NV12 mode reaches {} fps; "
                     "using {:.1f} fps. The strobe timing model assumes {} fps.",
                     REQUESTED_WIDTH, REQUESTED_HEIGHT, config_.targetFps, chosen->fps,
                     config_.targetFps);
    }

    ComPtr<IMFMediaType> pSelectedType;
    hr = sourceReader_->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                                           chosen->index, &pSelectedType);
    if (FAILED(hr) || !pSelectedType) return false;
    isNV12_ = (chosen->subtype == MFVideoFormat_NV12);

    hr = sourceReader_->SetCurrentMediaType(
        MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, pSelectedType.Get()
    );
    if (FAILED(hr)) return false;

    // Read the rate back from the type the reader actually settled on
    negotiatedFps_ = chosen->fps;
    ComPtr<IMFMediaType> pCurrent;
    if (SUCCEEDED(sourceReader_->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, &pCurrent))) {
        UINT32 num = 0, den = 0;
        if (SUCCEEDED(MFGetAttributeRatio(pCurrent.Get(), MF_MT_FRAME_RATE, &num, &den)) && den > 0) {
            negotiatedFps_ = static_cast<double>(num) / den;
        }
    }
    spdlog::info("[MediaFoundationDriver] Selected {} {}x{} @ {:.1f} fps (requested {} fps)",
                 subtypeName(chosen->subtype), chosen->width, chosen->height,
                 negotiatedFps_, config_.targetFps);

    // -------------------------------------------------------------------------
    // Extract frame properties
    // -------------------------------------------------------------------------
    UINT32 width = 0, height = 0;
    hr = MFGetAttributeSize(pSelectedType.Get(), MF_MT_FRAME_SIZE, &width, &height);
    if (FAILED(hr) || width == 0 || height == 0) {
        std::cerr << "[MediaFoundationDriver] Failed to read frame dimensions." << std::endl;
        return false;
    }
    frameWidth_  = width;
    frameHeight_ = height;

    // Stride can be negative (bottom-up DIB) — use signed type
    // MF_MT_DEFAULT_STRIDE is a LONG (signed 32-bit)
    LONG stride = 0;
    hr = pSelectedType->GetUINT32(MF_MT_DEFAULT_STRIDE, reinterpret_cast<UINT32*>(&stride));
    if (FAILED(hr) || stride == 0) {
        // Calculate minimum stride for the given format
        hr = MFGetStrideForBitmapInfoHeader(
            isNV12_ ? MFVideoFormat_NV12.Data1 : MFVideoFormat_L8.Data1,
            frameWidth_,
            &stride
        );
        if (FAILED(hr)) {
            stride = static_cast<LONG>(frameWidth_);  // Absolute last resort
        }
    }
    frameStride_ = stride;  // Preserve sign for cv::Mat step parameter

    return true;
}

// =============================================================================
// Internal: Extension Unit discovery via IKsTopologyInfo
// =============================================================================

bool MediaFoundationDriver::discoverExtensionUnit() {
    if (!mediaSource_) return false;

    ComPtr<IKsTopologyInfo> ksTopology;
    HRESULT hr = mediaSource_->QueryInterface(IID_PPV_ARGS(&ksTopology));
    if (FAILED(hr)) {
        spdlog::warn("[MediaFoundationDriver] Failed to QueryInterface for IKsTopologyInfo. HR=0x{:08X}",
                     static_cast<unsigned long>(hr));
        return false;
    }

    DWORD numNodes = 0;
    hr = ksTopology->get_NumNodes(&numNodes);
    if (FAILED(hr) || numNodes == 0) {
        spdlog::warn("[MediaFoundationDriver] IKsTopologyInfo get_NumNodes returned 0 or error. HR=0x{:08X}",
                     static_cast<unsigned long>(hr));
        return false;
    }

    spdlog::info("[MediaFoundationDriver] Inspecting {} UVC Topology Nodes for Extension Units...", numNodes);

    for (DWORD i = 0; i < numNodes; ++i) {
        GUID nodeType = GUID_NULL;
        hr = ksTopology->get_NodeType(i, &nodeType);
        if (FAILED(hr)) continue;

        spdlog::info("[MediaFoundationDriver] Node #{}: Type={:08X}-{:04X}-{:04X}",
                     i, nodeType.Data1, nodeType.Data2, nodeType.Data3);

        if (nodeType == KSNODETYPE_DEV_SPECIFIC) {
            ComPtr<IKsControl> tempKsControl;
            hr = ksTopology->CreateNodeInstance(i, IID_PPV_ARGS(&tempKsControl));
            if (SUCCEEDED(hr) && tempKsControl) {
                spdlog::info("[MediaFoundationDriver] Successfully created IKsControl instance for Node #{}!", i);
                xuNodeId_ = i;
                ksControl_ = tempKsControl;
                
                // Set up KSPROPERTY structure template for node instance IKsControl
                ZeroMemory(&cachedKspProp_, sizeof(cachedKspProp_));
                cachedKspProp_.Set   = ARDUCAM_XU_GUID;
                cachedKspProp_.Id    = 1;
                cachedKspProp_.Flags = KSPROPERTY_TYPE_SET;

                // Query the actual Extension Unit GUID to log it
                static const GUID MY_PROPSETID_VIDCAP_EXTENSION_UNIT = { 0x1C31960E, 0x7C1C, 0x4E2A, { 0x8B, 0xA6, 0x25, 0x8F, 0x2D, 0xD3, 0x4E, 0xEB } };
                KSPROPERTY prop;
                prop.Set = MY_PROPSETID_VIDCAP_EXTENSION_UNIT;
                prop.Id = KSPROPERTY_EXTENSION_UNIT_INFO;
                prop.Flags = KSPROPERTY_TYPE_GET;
                
                GUID actualGuid = {0};
                ULONG bytesReturned = 0;
                HRESULT guidHr = ksControl_->KsProperty(&prop, sizeof(prop), &actualGuid, sizeof(actualGuid), &bytesReturned);
                if (SUCCEEDED(guidHr)) {
                    spdlog::info("[MediaFoundationDriver] ACTUAL Extension Unit GUID for Node #{}: {{{:08X}-{:04X}-{:04X}-{:02X}{:02X}-{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}}}",
                        i, actualGuid.Data1, actualGuid.Data2, actualGuid.Data3,
                        actualGuid.Data4[0], actualGuid.Data4[1], actualGuid.Data4[2], actualGuid.Data4[3],
                        actualGuid.Data4[4], actualGuid.Data4[5], actualGuid.Data4[6], actualGuid.Data4[7]);
                } else {
                    spdlog::warn("[MediaFoundationDriver] Failed to get ACTUAL Extension Unit GUID. HR=0x{:08X}", static_cast<unsigned long>(guidHr));
                }

                return true;
            } else {
                spdlog::warn("[MediaFoundationDriver] CreateNodeInstance failed for Node #{}. HR=0x{:08X}",
                             i, static_cast<unsigned long>(hr));
            }
        }
    }
    spdlog::warn("[MediaFoundationDriver] No KSNODETYPE_DEV_SPECIFIC extension unit found among {} nodes.", numNodes);
    return false;
}

// =============================================================================
// HOT PATH: grabRawFrame() — called at 120+ FPS by the producer thread
// =============================================================================
// PERFORMANCE CONTRACT:
//   - Zero heap allocations (caller provides pre-allocated destination)
//   - Single memcpy from MF buffer into destination cv::Mat
//   - No COM QueryInterface calls
//   - No string operations or logging on the success path

bool MediaFoundationDriver::grabRawFrame(cv::Mat& destination) {
    if (!sourceReader_) return false;

    DWORD streamIndex = 0;
    DWORD flags = 0;
    LONGLONG timestamp = 0;
    ComPtr<IMFSample> pSample;

    HRESULT hr = sourceReader_->ReadSample(
        MF_SOURCE_READER_FIRST_VIDEO_STREAM,
        0,          // No flags — synchronous blocking read
        &streamIndex,
        &flags,
        &timestamp,
        &pSample
    );

    if (FAILED(hr) || !pSample) {
        return false;
    }

    // Check for stream discontinuity or format change
    if (flags & MF_SOURCE_READERF_ERROR) {
        return false;
    }

    ComPtr<IMFMediaBuffer> pBuffer;
    hr = pSample->ConvertToContiguousBuffer(&pBuffer);
    if (FAILED(hr)) return false;

    BYTE* pData = nullptr;
    DWORD maxLen = 0, curLen = 0;
    hr = pBuffer->Lock(&pData, &maxLen, &curLen);
    if (FAILED(hr) || !pData) return false;

    // Validate buffer has enough data for one frame's Y-plane
    const DWORD requiredBytes = static_cast<DWORD>(
        std::abs(frameStride_) * frameHeight_
    );
    if (curLen < requiredBytes) {
        pBuffer->Unlock();
        return false;
    }

    // Handle negative stride (bottom-up layout): adjust pointer to last row
    BYTE* pScanline0 = pData;
    size_t absStride = static_cast<size_t>(std::abs(frameStride_));
    if (frameStride_ < 0) {
        pScanline0 = pData + (frameHeight_ - 1) * absStride;
    }

    // Copy directly into caller's pre-allocated destination — single memcpy,
    // zero heap allocation. The destination is a pre-allocated cv::Mat slot
    // inside the AtomicRingBuffer's FrameSet.
    const cv::Mat wrapper(
        static_cast<int>(frameHeight_),
        static_cast<int>(frameWidth_),
        CV_8UC1,
        pScanline0,
        absStride
    );
    wrapper.copyTo(destination);
    lastTimestampUs_ = static_cast<uint64_t>(timestamp / 10);  // MF sample time is in 100 ns units

    // Unlock ASAP — release the USB/DMA buffer back to the driver
    pBuffer->Unlock();
    return true;
}

// =============================================================================
// setHardwareExposure — called during mode transitions, NOT on the hot path
// =============================================================================

void MediaFoundationDriver::setHardwareExposure(int microseconds) {
    if (!cameraControl_ || microseconds < 0) return;

    // UVC CameraControl_Exposure is log2(seconds): -9 = 1953 us, -7 = 7812 us.
    // Round to the nearest step (not floor) and clamp to what the device reports.
    long value = CameraConfig::exposureUsToLog2(microseconds);
    if (value < exposureLog2Min_) value = exposureLog2Min_;
    if (value > exposureLog2Max_) value = exposureLog2Max_;

    HRESULT hr = cameraControl_->Set(CameraControl_Exposure, value, CameraControl_Flags_Manual);
    if (FAILED(hr)) {
        spdlog::warn("[MediaFoundationDriver] Exposure set failed (HR=0x{:08X})",
                     static_cast<unsigned long>(hr));
        return;
    }

    long applied = value, flags = 0;
    if (SUCCEEDED(cameraControl_->Get(CameraControl_Exposure, &applied, &flags))) {
        value = applied;
    }
    appliedExposureUs_ = CameraConfig::exposureLog2ToUs(static_cast<int>(value));
    spdlog::info("[MediaFoundationDriver] Exposure requested {} us -> applied {} us (log2 {})",
                 microseconds, appliedExposureUs_, value);
}

void MediaFoundationDriver::setHardwareGain(int gain) {
    if (!procAmp_) return;
    long value = static_cast<long>(gain);
    if (value < gainMin_) value = gainMin_;
    if (value > gainMax_) value = gainMax_;

    HRESULT hr = procAmp_->Set(VideoProcAmp_Gain, value, VideoProcAmp_Flags_Manual);
    if (FAILED(hr)) {
        spdlog::warn("[MediaFoundationDriver] Gain set failed (HR=0x{:08X})",
                     static_cast<unsigned long>(hr));
        return;
    }
    long applied = value, flags = 0;
    if (SUCCEEDED(procAmp_->Get(VideoProcAmp_Gain, &applied, &flags))) {
        value = applied;
    }
    appliedGain_ = static_cast<int>(value);
    spdlog::info("[MediaFoundationDriver] Gain requested {} -> applied {}", gain, appliedGain_);
}

void MediaFoundationDriver::setHardwareBrightness(int level) {
    if (!procAmp_) return;
    long value = static_cast<long>(level);
    if (value < brightnessMin_) value = brightnessMin_;
    if (value > brightnessMax_) value = brightnessMax_;

    HRESULT hr = procAmp_->Set(VideoProcAmp_Brightness, value, VideoProcAmp_Flags_Manual);
    if (FAILED(hr)) {
        spdlog::warn("[MediaFoundationDriver] Brightness set failed (HR=0x{:08X})",
                     static_cast<unsigned long>(hr));
        return;
    }
    spdlog::info("[MediaFoundationDriver] Brightness (black level) set to {}", value);
}

void MediaFoundationDriver::setAutoExposure(bool enabled) {
    if (!cameraControl_) return;
    // The Manual/Auto flag rides along with a value; keep whatever is current.
    long value = 0, flags = 0;
    if (FAILED(cameraControl_->Get(CameraControl_Exposure, &value, &flags))) {
        value = exposureLog2Max_;
    }
    HRESULT hr = cameraControl_->Set(CameraControl_Exposure, value,
                                     enabled ? CameraControl_Flags_Auto : CameraControl_Flags_Manual);
    if (FAILED(hr)) {
        spdlog::warn("[MediaFoundationDriver] Auto-exposure {} failed (HR=0x{:08X})",
                     enabled ? "enable" : "disable", static_cast<unsigned long>(hr));
    }
}

void MediaFoundationDriver::setAutoGain(bool enabled) {
    if (!procAmp_) return;
    long value = 0, flags = 0;
    if (FAILED(procAmp_->Get(VideoProcAmp_Gain, &value, &flags))) {
        value = gainMin_;
    }
    HRESULT hr = procAmp_->Set(VideoProcAmp_Gain, value,
                               enabled ? VideoProcAmp_Flags_Auto : VideoProcAmp_Flags_Manual);
    if (FAILED(hr)) {
        // Many UVC cameras have no auto-gain flag at all; that is fine.
        spdlog::debug("[MediaFoundationDriver] Auto-gain {} not supported (HR=0x{:08X})",
                      enabled ? "enable" : "disable", static_cast<unsigned long>(hr));
    }
}

// =============================================================================
// Control interface caching / startup configuration
// =============================================================================

void MediaFoundationDriver::cacheControlInterfaces() {
    if (!mediaSource_) return;

    HRESULT hr = mediaSource_->QueryInterface(IID_PPV_ARGS(&cameraControl_));
    if (SUCCEEDED(hr) && cameraControl_) {
        long mn = 0, mx = 0, step = 0, def = 0, flags = 0;
        if (SUCCEEDED(cameraControl_->GetRange(CameraControl_Exposure, &mn, &mx, &step, &def, &flags))) {
            exposureLog2Min_ = mn;
            exposureLog2Max_ = mx;
            spdlog::info("[MediaFoundationDriver] Exposure range: log2 {}..{} ({}..{} us), default {}",
                         mn, mx, CameraConfig::exposureLog2ToUs(static_cast<int>(mn)),
                         CameraConfig::exposureLog2ToUs(static_cast<int>(mx)), def);
        }
    } else {
        spdlog::warn("[MediaFoundationDriver] IAMCameraControl unavailable; exposure cannot be set.");
    }

    hr = mediaSource_->QueryInterface(IID_PPV_ARGS(&procAmp_));
    if (SUCCEEDED(hr) && procAmp_) {
        long mn = 0, mx = 0, step = 0, def = 0, flags = 0;
        if (SUCCEEDED(procAmp_->GetRange(VideoProcAmp_Gain, &mn, &mx, &step, &def, &flags))) {
            gainMin_ = mn;
            gainMax_ = mx;
            spdlog::info("[MediaFoundationDriver] Gain range: {}..{}, default {}", mn, mx, def);
        }
        if (SUCCEEDED(procAmp_->GetRange(VideoProcAmp_Brightness, &mn, &mx, &step, &def, &flags))) {
            brightnessMin_ = mn;
            brightnessMax_ = mx;
            spdlog::info("[MediaFoundationDriver] Brightness range: {}..{}, default {}", mn, mx, def);
        }
    } else {
        spdlog::warn("[MediaFoundationDriver] IAMVideoProcAmp unavailable; gain/brightness cannot be set.");
    }
}

void MediaFoundationDriver::applyStartupConfig() {
    applyCameraConfig(config_);
    spdlog::info("[MediaFoundationDriver] Device {} configured: exposure {} us, gain {}, {:.1f} fps",
                 deviceIndex_, appliedExposureUs_, appliedGain_, negotiatedFps_);
}

// =============================================================================
// Native media type enumeration / selection
// =============================================================================

std::vector<MediaFoundationDriver::MediaTypeInfo>
MediaFoundationDriver::enumerateNativeMediaTypes() const {
    std::vector<MediaTypeInfo> types;
    if (!sourceReader_) return types;

    ComPtr<IMFMediaType> pType;
    for (DWORD i = 0;
         SUCCEEDED(sourceReader_->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, i, &pType));
         ++i, pType.Reset()) {
        GUID major = GUID_NULL;
        pType->GetGUID(MF_MT_MAJOR_TYPE, &major);
        if (major != MFMediaType_Video) continue;

        MediaTypeInfo info;
        info.index = i;
        pType->GetGUID(MF_MT_SUBTYPE, &info.subtype);
        MFGetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, &info.width, &info.height);
        UINT32 num = 0, den = 0;
        if (SUCCEEDED(MFGetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, &num, &den)) && den > 0) {
            info.fps = static_cast<double>(num) / den;
        }
        info.usable = (info.subtype == MFVideoFormat_L8 || info.subtype == MFVideoFormat_NV12);
        types.push_back(info);
    }
    return types;
}

const MediaFoundationDriver::MediaTypeInfo*
MediaFoundationDriver::selectMediaType(const std::vector<MediaTypeInfo>& types,
                                       uint32_t width, uint32_t height, int targetFps) {
    // Rank: usable subtype, then the requested size, then a rate that meets the
    // target (closest from above), then L8 over NV12, then the highest rate.
    const MediaTypeInfo* best = nullptr;
    auto rank = [&](const MediaTypeInfo& t) {
        const bool sizeOk   = (t.width == width && t.height == height);
        const bool meetsFps = (t.fps + 0.5 >= targetFps);
        const bool isL8     = (t.subtype == MFVideoFormat_L8);
        // Closest-from-above when the target is met; otherwise higher is better.
        const double fpsKey = meetsFps ? -(t.fps - targetFps) : t.fps;
        return std::make_tuple(sizeOk, meetsFps, isL8, fpsKey);
    };
    for (const auto& t : types) {
        if (!t.usable) continue;
        if (!best || rank(t) > rank(*best)) best = &t;
    }
    if (best && (best->width != width || best->height != height)) {
        spdlog::warn("[MediaFoundationDriver] No {}x{} L8/NV12 mode advertised; using {}x{}. "
                     "Upstream buffers are sized for {}x{}.",
                     width, height, best->width, best->height, width, height);
    }
    return best;
}

// =============================================================================
// HOT PATH: injectImmediateRegisterWrite() — microsecond-critical I2C command
// =============================================================================
// PERFORMANCE CONTRACT:
//   - Zero heap allocations
//   - No COM QueryInterface (IKsControl cached at init)
//   - No KSP_NODE construction (template cached at init)
//   - Single kernel transition via KsProperty()
//   - No logging on the success path (cerr only on failure)

void MediaFoundationDriver::injectImmediateRegisterWrite(uint16_t reg, uint8_t value) {
    // Disabled: The standard UVC firmware on this Arducam OV9281 bridge does not 
    // expose a UVC Extension Unit for I2C pass-through. KsProperty requests will 
    // fail with 0x80070492 (ERROR_PROP_NOT_FOUND).
}

#endif
