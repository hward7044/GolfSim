#pragma once
#include "IBufferManager.hpp"
#include <array>
class AtomicRingBuffer : public IBufferManager {
private:
    static constexpr size_t SIZE = 16;
    std::array<FrameSet, SIZE> buffer;
public:
    void push(const FrameSet& set) override;
};
