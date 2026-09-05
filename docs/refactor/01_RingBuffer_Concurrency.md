# Refactor 01: Ring Buffer & Concurrency Safety

This document details the issues, mathematical/thread concurrency models, and implementation specifications for `AtomicRingBuffer` and asynchronous diagnostics I/O.

---

## 1. Problem Statement

### 1.1 Unsynchronized Data Races on Overwrite
In high-speed tracking environments, an overwrite ring buffer policy is desired: if the consumer falls behind, the newest frame should overwrite the oldest unread frame rather than stalling the camera pipeline.

In the current [AtomicRingBuffer.hpp](file:///home/hward/Projects/GolfSim/include/Math/AtomicRingBuffer.hpp):
```cpp
void push(const T& item) override {
    const auto h = head.load(std::memory_order_relaxed);
    buffer[h & (N - 1)] = item; // Unconditional write
    head.store(h + 1, std::memory_order_release);
}
```
When full ($h - t = N$):
1. Consumer thread is currently executing `pop()` on slot `t & (N - 1)`.
2. Producer writes into slot `h & (N - 1)`.
3. Because $h \equiv t \pmod N$, both threads access the exact same memory slot simultaneously!
4. Since `cv::Mat` is not atomic, concurrent read/write invokes **Undefined Behavior (data race)**, corrupting internal matrix headers, reference counts, and row pointers.
5. Furthermore, `tail` is never updated by `push()`, causing $h - t > N$ and desynchronizing index arithmetic.

### 1.2 The `std::move` Zero-Allocation Violation
In `pop()`:
```cpp
T result = std::move(buffer[t & (N - 1)]);
```
- `FrameSet` contains `std::array<cv::Mat, 2> frames`.
- `cv::Mat(cv::Mat&&)` nullifies the source data pointer (`data = nullptr`, `rows = 0`, `cols = 0`).
- Once moved, the slot inside `buffer` holds an empty matrix.
- The next time the producer writes into that slot, `buffer[slot] = item;` calls `malloc()` / `new` to allocate a fresh image buffer on the OS heap.
- This creates continuous allocation and deallocation churn on the high-speed capture thread.

### 1.3 Consumer Thread Stalling via Synchronous File I/O
In `SessionStateMachine::processNextFrame()`:
- When stream recording is enabled, it calls `recorder.saveStreamSession(streamFrames)`.
- `saveStreamSession()` performs synchronous PNG encoding and file writes on the calling thread for 50 frames.
- This freezes the consumer thread for 200–500 ms while the producer thread continues pushing frames at 100 FPS, guaranteeing buffer overrun and dropped frames.

---

## 2. Technical Architecture & Design Solution

```
Producer (Camera Thread) ──> [Atomic Circular Overwrite Buffer] ──> Consumer (SessionStateMachine)
                                  │                                           │
                        • Atomically advances tail if full                    ▼
                        • Cache-line separated (alignas(64))        Async SaveTask Queue
                        • Zero heap allocation (copyTo / swap)                │
                                                                              ▼
                                                                  Background Worker Thread
                                                                  (Disk I/O & PNG encoding)
```

### 2.1 Safe Lock-Free Overwrite Circular Buffer Specification
To achieve lock-free overwrite semantics without data races:
1. **Full-Buffer Detection & Tail Advancement**:
   Before writing to `buffer[h & (N - 1)]`, the producer checks if $(h - t) \ge N$.
   If full, the producer advances `tail` to $h - N + 1$ using `compare_exchange_weak` or atomic store.
   This guarantees the consumer will not read the slot currently being overwritten.
2. **Preserving Pre-allocated Memory (`pop_into`)**:
   Instead of `std::optional<T> pop()`, we provide:
   ```cpp
   bool pop_into(T& destination);
   ```
   The consumer thread maintains its own pre-allocated `FrameSet`. Frame data is copied using `cv::Mat::copyTo(destination)`, which reuses destination memory without reallocation:
   $$\text{Pre-allocated Matrix Size} = 1280 \times 800 \times 1\text{ byte} = 1.024\text{ MB}$$
   Alternatively, we can swap matrix pointers using an auxiliary slot, maintaining zero-copy speed with zero allocations.
3. **Cache-Line Padding (`alignas(64)`)**:
   `head` and `tail` must reside on separate 64-byte L1 cache lines to prevent CPU core false sharing between producer and consumer cores:
   ```cpp
   alignas(64) std::atomic<std::size_t> head_{0};
   alignas(64) std::atomic<std::size_t> tail_{0};
   ```

### 2.2 Asynchronous Diagnostics Stream Saving
In `FlightRecorder`:
- Both `saveSession()` (single shot) and `saveStreamSession()` (continuous stream chunks) must enqueue a lightweight `SaveTask` into `taskQueue`.
- The background `workerThread` performs all `cv::imwrite` operations and directory management.
- The consumer thread returns immediately in $< 50\text{ \mu s}$.

---

## 3. Detailed Implementation Plan

### Step 1: Refactor `include/Math/AtomicRingBuffer.hpp`
- Implement `bool push_overwrite(const T& item)`.
- Implement `bool pop_into(T& dest)`.
- Enforce $N$ power-of-2 via `static_assert((N & (N - 1)) == 0)`.
- Add cache-line alignment to atomic indices.

### Step 2: Refactor `src/Diagnostics/FlightRecorder.cpp`
- Update `saveStreamSession` to package frames into `SaveTask` and enqueue to `taskQueue`.
- Eliminate synchronous disk operations from `SessionStateMachine.hpp`.

### Step 3: Verification
- Write a high-concurrency stress test pushing 20,000 frames with a slow consumer.
- Verify zero crashes, zero data races (via ThreadSanitizer), and zero heap allocations inside the loop.

