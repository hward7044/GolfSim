#pragma once
#include "FrameSet.hpp"
class IBufferManager {
public:
    virtual ~IBufferManager() = default;
    virtual void push(const FrameSet& set) = 0;
};
