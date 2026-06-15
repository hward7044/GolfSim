#include "PlaybackCameraNode.hpp"
cv::Mat PlaybackCameraNode::captureFrame() { return cv::Mat(); }
CameraRole PlaybackCameraNode::getRole() { return CameraRole::STEREO_LEFT; }
