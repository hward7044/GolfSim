#include "HAL/SerialPort.hpp"
#include <spdlog/spdlog.h>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstring>
#endif

SerialPort::~SerialPort() {
    close();
}

#ifdef _WIN32

bool SerialPort::open(const std::string& portName, int baudRate) {
    close();

    portName_ = portName;
    baudRate_ = baudRate;

    // Format port path (e.g. "COM3" -> "\\\\.\\COM3") to support ports > COM9
    std::string formattedPort = portName;
    if (formattedPort.find("\\\\.\\") == std::string::npos) {
        formattedPort = "\\\\.\\" + formattedPort;
    }

    HANDLE h = CreateFileA(
        formattedPort.c_str(),
        GENERIC_WRITE | GENERIC_READ,
        0,              // Exclusive access
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr
    );

    if (h == INVALID_HANDLE_VALUE) {
        DWORD err = GetLastError();
        spdlog::debug("[SerialPort] Failed to open Windows port {}. Error code: {}", portName, err);
        return false;
    }

    hSerial_ = h;

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(static_cast<HANDLE>(hSerial_), &dcbSerialParams)) {
        close();
        return false;
    }

    dcbSerialParams.BaudRate = static_cast<DWORD>(baudRate_);
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;
    dcbSerialParams.fOutxCtsFlow = FALSE;
    dcbSerialParams.fOutxDsrFlow = FALSE;
    dcbSerialParams.fDtrControl  = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl  = RTS_CONTROL_DISABLE;

    if (!SetCommState(static_cast<HANDLE>(hSerial_), &dcbSerialParams)) {
        close();
        return false;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 10;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(static_cast<HANDLE>(hSerial_), &timeouts);

    PurgeComm(static_cast<HANDLE>(hSerial_), PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
}

void SerialPort::close() {
    if (isOpen()) {
        CloseHandle(static_cast<HANDLE>(hSerial_));
        hSerial_ = (void*)-1;
    }
}

bool SerialPort::writeString(const std::string& data) {
    if (!isOpen()) return false;
    DWORD bytesWritten = 0;
    BOOL res = WriteFile(static_cast<HANDLE>(hSerial_), data.c_str(), static_cast<DWORD>(data.size()), &bytesWritten, nullptr);
    return (res && bytesWritten == data.size());
}

bool SerialPort::writeChar(char c) {
    if (!isOpen()) return false;
    DWORD bytesWritten = 0;
    BOOL res = WriteFile(static_cast<HANDLE>(hSerial_), &c, 1, &bytesWritten, nullptr);
    return (res && bytesWritten == 1);
}

void SerialPort::flush() {
    if (isOpen()) {
        FlushFileBuffers(static_cast<HANDLE>(hSerial_));
    }
}

bool SerialPort::isOpen() const noexcept {
    return hSerial_ != (void*)-1;
}

#else // POSIX / Linux

static speed_t getPosixBaud(int baudRate) {
    switch (baudRate) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
        default:     return B115200;
    }
}

bool SerialPort::open(const std::string& portName, int baudRate) {
    close();

    portName_ = portName;
    baudRate_ = baudRate;

    fd_ = ::open(portName_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        spdlog::debug("[SerialPort] Failed to open POSIX port {}: {}", portName, strerror(errno));
        return false;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }

    speed_t speed = getPosixBaud(baudRate_);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // non-blocking read
    tty.c_cc[VTIME] = 1;                        // 0.1 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;                    // no hardware flow control
#endif

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (isOpen()) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::writeString(const std::string& data) {
    if (!isOpen()) return false;
    ssize_t bytesWritten = ::write(fd_, data.data(), data.size());
    return bytesWritten == static_cast<ssize_t>(data.size());
}

bool SerialPort::writeChar(char c) {
    if (!isOpen()) return false;
    ssize_t bytesWritten = ::write(fd_, &c, 1);
    return bytesWritten == 1;
}

void SerialPort::flush() {
    if (isOpen()) {
        tcdrain(fd_);
    }
}

bool SerialPort::isOpen() const noexcept {
    return fd_ >= 0;
}

#endif

bool SerialPort::openWithRetry(const std::string& portName, int baudRate, int maxRetries, int delayMs) {
    for (int attempt = 0; attempt <= maxRetries; ++attempt) {
        if (open(portName, baudRate)) {
            spdlog::info("[SerialPort] Connected to {} at {} baud (attempt {}/{})",
                         portName, baudRate, attempt + 1, maxRetries + 1);
            return true;
        }
        if (attempt < maxRetries) {
            spdlog::warn("[SerialPort] Connection to {} failed (attempt {}/{}). Retrying in {}ms...",
                         portName, attempt + 1, maxRetries + 1, delayMs);
            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
    }
    spdlog::warn("[SerialPort] Could not open port '{}' after {} attempts. Continuing in offline/headless mode.",
                 portName, maxRetries + 1);
    return false;
}

