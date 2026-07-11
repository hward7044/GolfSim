#pragma once

#ifdef _WIN32
#include <windows.h>
#include <string>

class Win32Serial {
private:
    HANDLE hSerial_ = INVALID_HANDLE_VALUE;
    std::string portName_;
    int baudRate_ = 115200;

public:
    Win32Serial() = default;
    ~Win32Serial();

    // Prevent copying
    Win32Serial(const Win32Serial&) = delete;
    Win32Serial& operator=(const Win32Serial&) = delete;

    bool open(const std::string& portName, int baudRate = 115200);
    void close();
    bool writeString(const std::string& data);
    bool isOpen() const noexcept { return hSerial_ != INVALID_HANDLE_VALUE; }
};
#endif
