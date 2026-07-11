#pragma once
#include "Math/LaunchData.hpp"

class INetworkTransmitter {
public:
    virtual ~INetworkTransmitter() = default;
    virtual bool transmitLaunchData(const LaunchData<Degrees, MilesPerHour>& data) = 0;
};

