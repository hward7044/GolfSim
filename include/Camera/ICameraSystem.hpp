#pragma once
#include "Camera/FrameSet.hpp"
class ICameraSystem {
public:
    virtual ~ICameraSystem() = default;
    virtual FrameSet captureSynchronizedFrames() = 0;
};
