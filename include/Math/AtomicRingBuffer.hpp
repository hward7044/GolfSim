#pragma once
#include "Math/IBufferManager.hpp"
#include <array>
#include <optional>
#include <atomic>

// T.1 — raise level of abstraction
// template<typename T, std::size_t N>
// Caches burst of IR strobe frames lock-free.
template<typename T, std::size_t N>
class AtomicRingBuffer : public IBufferManager<T> {
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");
private:
    std::array<T, N> buffer;
    std::atomic<std::size_t> head{0};
    std::atomic<std::size_t> tail{0};
public:
    void push(const T& item) override {
        const auto h = head.load(std::memory_order_relaxed);
        // Copy data into pre-allocated slot (no allocation)
        buffer[h & (N - 1)] = item;
        head.store(h + 1, std::memory_order_release);
    }

    std::optional<T> pop() override {
        const auto t = tail.load(std::memory_order_relaxed);
        if (t == head.load(std::memory_order_acquire))
            return std::nullopt;
        T result = std::move(buffer[t & (N - 1)]);
        tail.store(t + 1, std::memory_order_release);
        return result;
    }
};
