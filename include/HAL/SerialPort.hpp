#pragma once

#include <string>
#include <chrono>

class SerialPort {
private:
#ifdef _WIN32
    void* hSerial_ = (void*)-1; // Windows HANDLE (INVALID_HANDLE_VALUE)
#else
    int fd_ = -1;                // POSIX file descriptor
#endif
    std::string portName_;
    int baudRate_ = 115200;

public:
    SerialPort() = default;
    ~SerialPort();

    // Non-copyable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Direct open
    bool open(const std::string& portName, int baudRate = 115200);

    // Open with automatic retries (e.g., maxRetries = 2 gives 3 total attempts)
    bool openWithRetry(const std::string& portName, int baudRate = 115200, int maxRetries = 2, int delayMs = 500);

    void close();
    bool writeString(const std::string& data);
    bool writeChar(char c);
    void flush();
    bool isOpen() const noexcept;

    const std::string& getPortName() const noexcept { return portName_; }
    int getBaudRate() const noexcept { return baudRate_; }
};

