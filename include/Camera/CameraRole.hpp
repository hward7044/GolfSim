#pragma once
#include <cstddef>

enum class CameraRole {
    STEREO_LEFT,
    STEREO_RIGHT,
    // --- Add new roles above this line ---
    COUNT  // Sentinel: must be last. Used to size FrameSet::frames.
};

constexpr std::size_t CAMERA_ROLE_COUNT = static_cast<std::size_t>(CameraRole::COUNT);
