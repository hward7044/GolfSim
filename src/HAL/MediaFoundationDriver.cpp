#include "HAL/MediaFoundationDriver.hpp"
#ifdef _WIN32
cv::Mat MediaFoundationDriver::grabRawFrame() { return cv::Mat(); }
void MediaFoundationDriver::setHardwareExposure(int microseconds) {}
void MediaFoundationDriver::injectImmediateRegisterWrite(uint16_t reg, uint8_t value) {}
#endif
