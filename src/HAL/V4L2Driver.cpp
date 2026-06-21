#include "HAL/V4L2Driver.hpp"
#ifdef __linux__
cv::Mat V4L2Driver::grabRawFrame() { return cv::Mat(); }
void V4L2Driver::setHardwareExposure(int microseconds) {}
void V4L2Driver::injectImmediateRegisterWrite(uint16_t reg, uint8_t value) {}
#endif
