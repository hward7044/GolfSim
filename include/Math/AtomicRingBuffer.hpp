#pragma once
#include "Math/IBufferManager.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <utility>
#include <thread>
#include <limits>

// T.1 — raise level of abstraction
// template<typename T, std::size_t N>
// Caches burst of IR strobe frames lock-free with safe overwrite semantics.
template<typename T, std::size_t N>
class AtomicRingBuffer : public IBufferManager<T> {
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");
    static_assert(N >= 2, "N must be at least 2");

private:
    static constexpr std::size_t NO_SLOT = std::numeric_limits<std::size_t>::max();

    struct Slot {
        T data;
    };

    std::array<Slot, N> buffer_;

    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
    alignas(64) std::atomic<std::size_t> active_slot_{NO_SLOT};

public:
    AtomicRingBuffer() = default;

    /// Pre-allocate all buffer slots (e.g. for FrameSet with OV9281 resolution)
    template<typename... Args>
    void preallocate(Args&&... args) {
        for (auto& slot : buffer_) {
            if constexpr (requires { slot.data.preallocate(std::forward<Args>(args)...); }) {
                slot.data.preallocate(std::forward<Args>(args)...);
            }
        }
    }

    /// Push an item into the ring buffer with overwrite policy.
    /// If full, advances tail to drop the oldest frame.
    /// Swaps item with the internal slot for zero-copy, zero-allocation transfer.
    void push(T& item) override {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        std::size_t t = tail_.load(std::memory_order_relaxed);

        // Maintain invariant: buffer capacity is N - 1 so producer never wraps onto reading consumer
        while (h >= t + (N - 1)) {
            const std::size_t oldestSlot = t & (N - 1);
            while (active_slot_.load(std::memory_order_acquire) == oldestSlot) {
                std::this_thread::yield();
            }
            if (tail_.compare_exchange_weak(t, t + 1, std::memory_order_acq_rel)) {
                t = t + 1;
                break;
            }
        }

        const std::size_t slotIdx = h & (N - 1);
        while (active_slot_.load(std::memory_order_acquire) == slotIdx) {
            std::this_thread::yield();
        }

        using std::swap;
        swap(buffer_[slotIdx].data, item);

        head_.store(h + 1, std::memory_order_release);
    }

    void push(T&& item) override {
        push(item);
    }

    /// Pop an item from the ring buffer into dest via zero-copy swap.
    /// Returns true on success, false if empty.
    bool pop(T& dest) override {
        std::size_t t = tail_.load(std::memory_order_relaxed);

        while (true) {
            const std::size_t h = head_.load(std::memory_order_acquire);
            if (t >= h) {
                return false; // Buffer is empty
            }

            const std::size_t slotIdx = t & (N - 1);

            // Mark slot as actively being read
            active_slot_.store(slotIdx, std::memory_order_release);

            // Verify tail didn't advance past t while marking
            const std::size_t currentTail = tail_.load(std::memory_order_acquire);
            if (currentTail > t) {
                active_slot_.store(NO_SLOT, std::memory_order_release);
                t = currentTail;
                continue;
            }

            // Reserve slot t by advancing tail
            if (tail_.compare_exchange_weak(t, t + 1, std::memory_order_acq_rel)) {
                using std::swap;
                swap(dest, buffer_[slotIdx].data);
                active_slot_.store(NO_SLOT, std::memory_order_release);
                return true;
            }

            active_slot_.store(NO_SLOT, std::memory_order_release);
        }
    }

    std::size_t size() const noexcept {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        return (h >= t) ? (h - t) : 0;
    }

    bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) <= tail_.load(std::memory_order_relaxed);
    }

    static constexpr std::size_t capacity() noexcept {
        return N - 1;
    }
};
