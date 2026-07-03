#include "HAL/Win32Serial.hpp"
#ifdef _WIN32

#include <spdlog/spdlog.h>
#include <sstream>

Win32Serial::~Win32Serial() {
    close();
}

bool Win32Serial::open(const std::string& portName, int baudRate) {
    close(); // Close any existing port

    portName_ = portName;
    baudRate_ = baudRate;

    // Format port path (e.g. "COM3" -> "\\\\.\\COM3") to support ports > COM9
    std::string formattedPort = portName;
    if (formattedPort.find("\\\\.\\") == std::string::npos) {
        formattedPort = "\\\\.\\" + formattedPort;
    }

    hSerial_ = CreateFileA(
        formattedPort.c_str(),
        GENERIC_WRITE,
        0,              // Shared mode: must be 0 for exclusive access to COM port
        nullptr,        // Security attributes
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr         // Template file
    );

    if (hSerial_ == INVALID_HANDLE_VALUE) {
        DWORD err = GetLastError();
        spdlog::error("[Win32Serial] Failed to open serial port {}. Error: {}", portName, err);
        return false;
    }

    // Configure DCB (Device Control Block) settings
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial_, &dcbSerialParams)) {
        DWORD err = GetLastError();
        spdlog::error("[Win32Serial] GetCommState failed for {}. Error: {}", portName, err);
        close();
        return false;
    }

    dcbSerialParams.BaudRate = static_cast<DWORD>(baudRate_);
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    // Disable hardware flow control and DTR/RTS assertions to avoid reboot loops
    dcbSerialParams.fOutxCtsFlow = FALSE;
    dcbSerialParams.fOutxDsrFlow = FALSE;
    dcbSerialParams.fDtrControl  = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl  = RTS_CONTROL_DISABLE;

    if (!SetCommState(hSerial_, &dcbSerialParams)) {
        DWORD err = GetLastError();
        spdlog::error("[Win32Serial] SetCommState failed for {}. Error: {}", portName, err);
        close();
        return false;
    }

    // Configure timeouts explicitly to avoid blocking indefinitely on WriteFile
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 50; // Allow max 50ms write blocking

    if (!SetCommTimeouts(hSerial_, &timeouts)) {
        DWORD err = GetLastError();
        spdlog::error("[Win32Serial] SetCommTimeouts failed for {}. Error: {}", portName, err);
        close();
        return false;
    }

    spdlog::info("[Win32Serial] Successfully opened serial port {} at {} baud.", portName, baudRate);
    return true;
}

void Win32Serial::close() {
    if (hSerial_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial_);
        hSerial_ = INVALID_HANDLE_VALUE;
        spdlog::info("[Win32Serial] Closed serial port {}.", portName_);
    }
}

bool Win32Serial::writeString(const std::string& data) {
    if (hSerial_ == INVALID_HANDLE_VALUE) {
        spdlog::error("[Win32Serial] Cannot write data. Port is not open.");
        return false;
    }

    DWORD bytesWritten = 0;
    BOOL result = WriteFile(
        hSerial_,
        data.c_str(),
        static_cast<DWORD>(data.length()),
        &bytesWritten,
        nullptr
    );

    if (!result) {
        DWORD err = GetLastError();
        spdlog::error("[Win32Serial] WriteFile failed. Error: {}", err);
        return false;
    }

    if (bytesWritten != data.length()) {
        spdlog::warn("[Win32Serial] Written bytes mismatch: expected {}, wrote {}.", data.length(), bytesWritten);
        return false;
    }

    return true;
}

#endif
