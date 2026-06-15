#include "MediaFoundationDriver.hpp"
#ifdef _WIN32
cv::Mat MediaFoundationDriver::grabRawFrame() { return cv::Mat(); }
void MediaFoundationDriver::setHardwareExposure(int microseconds) {}
#endif
