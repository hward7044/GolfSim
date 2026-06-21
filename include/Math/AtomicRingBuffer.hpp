#pragma once
#include "Math/IBufferManager.hpp"
#include <array>
#include <optional>

// T.1 — raise level of abstraction
// template<typename T, std::size_t N>
// Caches burst of IR strobe frames lock-free.
template<typename T, std::size_t N>
class AtomicRingBuffer : public IBufferManager<T> {
private:
    std::array<T, N> buffer;
    std::size_t head = 0;
    std::size_t tail = 0;
public:
    void push(T item) override {
        // Stub: write item into buffer at head, advance head
    }

    std::optional<T> pop() override {
        // Stub: return item at tail if available
        return std::nullopt;
    }
};
