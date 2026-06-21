#include "Camera/PlaybackCameraNode.hpp"
bool PlaybackCameraNode::captureFrame(cv::Mat& destination) { return false; }
CameraRole PlaybackCameraNode::getRole() { return CameraRole::STEREO_LEFT; }
