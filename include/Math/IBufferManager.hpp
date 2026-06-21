#pragma once
#include <optional>

// T.1 — raise level of abstraction
// Generic buffer interface; implementations may be lock-free ring buffers,
// deques, etc.
template<typename T>
class IBufferManager {
public:
    virtual ~IBufferManager() = default;
    virtual void push(T item) = 0;
    virtual std::optional<T> pop() = 0;
};
