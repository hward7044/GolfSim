#pragma once
#include "Math/INetworkTransmitter.hpp"
#include <string>

#ifdef _WIN32
using SocketType = unsigned __int64; // Equivalent to SOCKET (UINT_PTR) on Win64
#else
using SocketType = int;
#endif

class TcpJsonTransmitter : public INetworkTransmitter {
private:
    std::string ip_;
    int         port_;
    SocketType  socket_;
    bool        connected_;

    bool connectToServer();
    void disconnect();

public:
    TcpJsonTransmitter(const std::string& ip = "127.0.0.1", int port = 3111);
    ~TcpJsonTransmitter() override;

    bool transmitLaunchData(const LaunchData<Degrees, MilesPerHour>& data) override;
};

