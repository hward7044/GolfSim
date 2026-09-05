#pragma once

// T.1 — raise level of abstraction
// Generic buffer interface for lock-free frame caching.
template<typename T>
class IBufferManager {
public:
    virtual ~IBufferManager() = default;
    virtual void push(T& item) = 0;
    virtual void push(T&& item) { push(item); }
    virtual bool pop(T& dest) = 0;
};
