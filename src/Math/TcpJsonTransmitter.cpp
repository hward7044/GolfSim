#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "Math/TcpJsonTransmitter.hpp"
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#ifndef _WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

TcpJsonTransmitter::TcpJsonTransmitter(const std::string& ip, int port)
    : ip_(ip), port_(port), socket_(INVALID_SOCKET), connected_(false) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

TcpJsonTransmitter::~TcpJsonTransmitter() {
    disconnect();
#ifdef _WIN32
    WSACleanup();
#endif
}

void TcpJsonTransmitter::disconnect() {
    if (socket_ != INVALID_SOCKET) {
#ifdef _WIN32
        closesocket(socket_);
#else
        close(socket_);
#endif
        socket_ = INVALID_SOCKET;
    }
    connected_ = false;
}

bool TcpJsonTransmitter::connectToServer() {
    if (connected_) return true;

    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ == INVALID_SOCKET) {
        spdlog::error("[TcpJsonTransmitter] Failed to create socket.");
        return false;
    }

    sockaddr_in clientService;
    clientService.sin_family = AF_INET;
    clientService.sin_port = htons(port_);
    
#ifdef _WIN32
    inet_pton(AF_INET, ip_.c_str(), &clientService.sin_addr);
#else
    clientService.sin_addr.s_addr = inet_addr(ip_.c_str());
#endif

    // Set non-blocking mode
#ifdef _WIN32
    unsigned long mode = 1;
    ioctlsocket(socket_, FIONBIO, &mode);
#else
    int flags = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
#endif

    int connResult = connect(socket_, (sockaddr*)&clientService, sizeof(clientService));
    bool isPending = false;

#ifdef _WIN32
    if (connResult == SOCKET_ERROR && WSAGetLastError() == WSAEWOULDBLOCK) {
        isPending = true;
    }
#else
    if (connResult == SOCKET_ERROR && errno == EINPROGRESS) {
        isPending = true;
    }
#endif

    if (connResult == SOCKET_ERROR && !isPending) {
        disconnect();
        return false;
    }

    if (isPending) {
        // Wait up to 50ms for connection
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(socket_, &writefds);

        fd_set exceptfds;
        FD_ZERO(&exceptfds);
        FD_SET(socket_, &exceptfds);

        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000; // 50ms

        int selResult = select(static_cast<int>(socket_ + 1), nullptr, &writefds, &exceptfds, &tv);
        if (selResult <= 0 || FD_ISSET(socket_, &exceptfds)) {
            disconnect();
            return false;
        }

        // Check if there was a socket-level error
        int socketError = 0;
#ifdef _WIN32
        int len = sizeof(socketError);
        getsockopt(socket_, SOL_SOCKET, SO_ERROR, (char*)&socketError, &len);
#else
        socklen_t len = sizeof(socketError);
        getsockopt(socket_, SOL_SOCKET, SO_ERROR, &socketError, &len);
#endif
        if (socketError != 0) {
            disconnect();
            return false;
        }
    }

    // Restore blocking mode
#ifdef _WIN32
    mode = 0;
    ioctlsocket(socket_, FIONBIO, &mode);
#else
    fcntl(socket_, F_SETFL, flags);
#endif

    // Set send timeout to 50ms
    int timeout_ms = 50;
#ifdef _WIN32
    setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms * 1000;
    setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
#endif

    connected_ = true;
    spdlog::info("[TcpJsonTransmitter] Connected to simulator at {}:{}", ip_, port_);
    return true;
}

bool TcpJsonTransmitter::transmitLaunchData(const LaunchData<Degrees, MilesPerHour>& data) {
    if (!connected_) {
        // Attempt to connect (non-fatal if simulator is not listening yet)
        if (!connectToServer()) {
            spdlog::warn("[TcpJsonTransmitter] Failed to connect, cannot transmit launch data.");
            return false;
        }
    }

    nlohmann::json jsonPayload;
    jsonPayload["type"] = "shot";
    jsonPayload["ballSpeed"] = data.ballSpeed.value();
    jsonPayload["verticalLaunchAngle"] = data.verticalLaunchAngle.value();
    jsonPayload["horizontalLaunchAngle"] = data.horizontalLaunchAngle.value();
    jsonPayload["spinSpeed"] = data.spinRPM;
    jsonPayload["spinAxis"] = { data.spinAxis.x(), data.spinAxis.y(), data.spinAxis.z() };

    std::string serialized = jsonPayload.dump() + "\n";

    int bytesSent = send(socket_, serialized.c_str(), static_cast<int>(serialized.length()), 0);
    if (bytesSent == SOCKET_ERROR) {
        spdlog::error("[TcpJsonTransmitter] Send failed, disconnecting.");
        disconnect();
        return false;
    }

    spdlog::info("[TcpJsonTransmitter] Successfully transmitted launch telemetry ({} bytes).", bytesSent);
    return true;
}
