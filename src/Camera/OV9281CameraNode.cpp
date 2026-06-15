#include "Camera/OV9281CameraNode.hpp"
cv::Mat OV9281CameraNode::captureFrame() { return cv::Mat(); }
CameraRole OV9281CameraNode::getRole() { return CameraRole::STEREO_LEFT; }
