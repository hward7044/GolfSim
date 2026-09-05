#include "HAL/MediaFoundationDriver.hpp"
#ifdef _WIN32

#include <iostream>
#include <vector>
#include <cmath>
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

// =============================================================================
// Construction / Destruction
// =============================================================================

MediaFoundationDriver::MediaFoundationDriver(uint32_t deviceIndex)
    : deviceIndex_(deviceIndex) {}

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

    // Apply trigger and exposure configurations (non-blocking, failures logged as warnings)
    configureTriggerAndExposureSettings();

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
    // Format negotiation: prefer native L8 (monochrome), fallback to NV12
    // -------------------------------------------------------------------------
    DWORD mediaTypeIndex = 0;
    ComPtr<IMFMediaType> pNV12Type;   // Track best NV12 candidate
    ComPtr<IMFMediaType> pSelectedType;
    bool foundL8 = false;

    ComPtr<IMFMediaType> pCurrentType;
    while (SUCCEEDED(sourceReader_->GetNativeMediaType(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM, mediaTypeIndex, &pCurrentType))) {

        GUID majorType = GUID_NULL;
        GUID subType = GUID_NULL;
        pCurrentType->GetGUID(MF_MT_MAJOR_TYPE, &majorType);
        pCurrentType->GetGUID(MF_MT_SUBTYPE, &subType);

        if (majorType == MFMediaType_Video) {
            if (subType == MFVideoFormat_L8) {
                pSelectedType = pCurrentType;
                isNV12_ = false;
                foundL8 = true;
                break;  // L8 is ideal, stop searching
            } else if (subType == MFVideoFormat_NV12 && !pNV12Type) {
                pNV12Type = pCurrentType;  // Save first NV12 candidate
            }
        }

        pCurrentType.Reset();  // Release before next iteration
        mediaTypeIndex++;
    }

    // Use NV12 fallback if L8 not found
    if (!foundL8 && pNV12Type) {
        pSelectedType = pNV12Type;
        isNV12_ = true;
    }

    if (!pSelectedType) {
        std::cerr << "[MediaFoundationDriver] No compatible media type found (L8 or NV12)." << std::endl;
        return false;
    }

    hr = sourceReader_->SetCurrentMediaType(
        MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, pSelectedType.Get()
    );
    if (FAILED(hr)) return false;

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

    // Unlock ASAP — release the USB/DMA buffer back to the driver
    pBuffer->Unlock();
    return true;
}

// =============================================================================
// setHardwareExposure — called during mode transitions, NOT on the hot path
// =============================================================================

void MediaFoundationDriver::setHardwareExposure(int microseconds) {
    if (!mediaSource_ || microseconds < 0) return;

    // Strategy: Use IAMCameraControl for standard UVC exposure control.
    // UVC spec defines exposure in log-base-2 seconds:
    //   value = log2(exposure_seconds)
    //   e.g., -15 ≈ 30µs, -14 ≈ 61µs, -13 ≈ 122µs, -10 ≈ 976µs
    ComPtr<IAMCameraControl> cameraControl;
    HRESULT hr = mediaSource_->QueryInterface(IID_PPV_ARGS(&cameraControl));
    if (FAILED(hr)) return;

    // Convert microseconds to log2 seconds
    // exposure_seconds = microseconds * 1e-6
    // value = floor(log2(exposure_seconds)) = floor(log2(microseconds) - log2(1e6))
    // log2(1e6) ≈ 19.93
    long value;
    if (microseconds <= 0) {
        value = -15;  // Minimum exposure (~30µs)
    } else {
        double log2Val = std::log2(static_cast<double>(microseconds)) - 19.931568;
        value = static_cast<long>(std::floor(log2Val));
        // Clamp to reasonable UVC range
        if (value < -15) value = -15;
        if (value > -1)  value = -1;
    }

    cameraControl->Set(CameraControl_Exposure, value, CameraControl_Flags_Manual);
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

void MediaFoundationDriver::configureTriggerAndExposureSettings() {
    // Disabled: Standard UVC driver handles exposure. We cannot write I2C registers 
    // via UVC on this hardware, so we rely on the default firmware state.
    spdlog::info("[MediaFoundationDriver] Hardware register configuration skipped (Unsupported on this UVC bridge).");
}

#endif
