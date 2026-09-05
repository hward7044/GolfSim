#include <windows.h>
#include <iostream>
#include <mfapi.h>
#include <mfplay.h>
#include <mfreadwrite.h>
#include <ks.h>
#include <ksmedia.h>
#include <vidcap.h>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "strmiids.lib")

int main() {
    CoInitializeEx(NULL, COINIT_MULTITHREADED);
    MFStartup(MF_VERSION);
    IMFAttributes* pAttributes = NULL;
    MFCreateAttributes(&pAttributes, 1);
    pAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
    
    IMFActivate** ppDevices = NULL;
    UINT32 count = 0;
    MFEnumDeviceSources(pAttributes, &ppDevices, &count);
    
    for (UINT32 i = 0; i < count; i++) {
        IMFMediaSource* pSource = NULL;
        if (SUCCEEDED(ppDevices[i]->ActivateObject(IID_PPV_ARGS(&pSource)))) {
            IKsTopologyInfo* pTopologyInfo = NULL;
            if (SUCCEEDED(pSource->QueryInterface(IID_PPV_ARGS(&pTopologyInfo)))) {
                DWORD numNodes = 0;
                pTopologyInfo->get_NumNodes(&numNodes);
                for (DWORD j = 0; j < numNodes; j++) {
                    GUID nodeType;
                    pTopologyInfo->get_NodeType(j, &nodeType);
                    if (nodeType == KSNODETYPE_DEV_SPECIFIC) {
                        IKsControl* pControl = NULL;
                        if (SUCCEEDED(pTopologyInfo->CreateNodeInstance(j, IID_PPV_ARGS(&pControl)))) {
                            KSPROPERTY prop;
                            prop.Set = PROPSETID_VIDCAP_EXTENSION_UNIT;
                            prop.Id = KSPROPERTY_EXTENSION_UNIT_INFO;
                            prop.Flags = KSPROPERTY_TYPE_GET;
                            
                            GUID xuGuid = {0};
                            DWORD bytesReturned = 0;
                            HRESULT hr = pControl->KsProperty(&prop, sizeof(prop), &xuGuid, sizeof(xuGuid), &bytesReturned);
                            if (SUCCEEDED(hr)) {
                                printf("Found Extension Unit GUID: {%08X-%04X-%04X-%02X%02X-%02X%02X%02X%02X%02X%02X}\n",
                                    xuGuid.Data1, xuGuid.Data2, xuGuid.Data3,
                                    xuGuid.Data4[0], xuGuid.Data4[1], xuGuid.Data4[2], xuGuid.Data4[3],
                                    xuGuid.Data4[4], xuGuid.Data4[5], xuGuid.Data4[6], xuGuid.Data4[7]);
                            } else {
                                printf("Failed to get XU GUID on Node %d. HR=0x%08X\n", j, hr);
                            }
                            pControl->Release();
                        }
                    }
                }
                pTopologyInfo->Release();
            }
            pSource->Release();
        }
        ppDevices[i]->Release();
    }
    CoTaskMemFree(ppDevices);
    pAttributes->Release();
    MFShutdown();
    CoUninitialize();
    return 0;
}
