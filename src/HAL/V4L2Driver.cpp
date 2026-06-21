#include "HAL/V4L2Driver.hpp"
#ifdef __linux__
bool V4L2Driver::grabRawFrame(cv::Mat& destination) { return false; }
void V4L2Driver::setHardwareExposure(int microseconds) {}
void V4L2Driver::injectImmediateRegisterWrite(uint16_t reg, uint8_t value) {}
#endif
