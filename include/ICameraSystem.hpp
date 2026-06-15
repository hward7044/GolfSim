#pragma once
#include "FrameSet.hpp"
class ICameraSystem {
public:
    virtual ~ICameraSystem() = default;
    virtual FrameSet captureSynchronizedFrames() = 0;
};
