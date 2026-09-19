# Refactor 08: Linux V4L2 Camera Driver

This document describes the Linux camera backend that brings the Arch/Omarchy build to parity with the Windows Media Foundation path, so `--live`, `--stream`, and the production launch-monitor pipeline all run on Linux against the same OV9281 UVC cameras.

---

## 1. Problem Statement

Before this refactor `V4L2Driver` was a three-line stub and `main()` returned at camera initialization on any non-Windows platform. The POSIX serial path (`SerialPort.cpp`, termios) was already complete, so the missing piece was purely video capture.

**Design decision**: raw V4L2 `ioctl` with MMAP streaming, not `cv::VideoCapture(CAP_V4L2)`. This matches the hand-rolled Media Foundation driver and gives explicit control over pixel format, queue depth, manual exposure, kernel frame timestamps, and a blocking read that another thread can interrupt. No new link dependencies — only `<linux/videodev2.h>`.

---

## 2. Platform Selection

`main.cpp` no longer branches on `_WIN32` around camera setup. Both drivers expose the same public surface, selected once by alias:

```cpp
#ifdef _WIN32
using PlatformCameraDriver = MediaFoundationDriver;
#else
using PlatformCameraDriver = V4L2Driver;
#endif
```

| Member | Purpose |
| :--- | :--- |
| `explicit Driver(uint32_t logicalIndex)` | Nth camera as the platform enumerates it |
| `bool initialize()` | Open, negotiate, configure, start streaming |
| `void shutdown()` | Idempotent teardown; unblocks a pending grab |
| `getFrameWidth()` / `getFrameHeight()` | Resolved geometry for `FrameSet::preallocate` |
| `static logConnectedDevices()` | Enumeration printout at startup |
| `grabRawFrame(cv::Mat&)` | Hot-path strided copy into the caller's buffer |
| `setHardwareExposure(int us)` | Manual exposure for the `--live` `e` key |
| `getLastFrameTimestampUs()` | Kernel/MF capture time of the last frame |

`V4L2Driver` additionally accepts an explicit device path (`--left-dev /dev/video2`).

---

## 3. Device Index Mapping

The `uvcvideo` kernel driver creates **two** `/dev/video*` nodes per UVC camera: a video capture node and a metadata node. Naive index → `/dev/videoN` mapping would therefore put the two cameras at `video0` and `video2`.

`V4L2Driver::enumerateCaptureDevices()` filters to nodes that report `V4L2_CAP_VIDEO_CAPTURE` **and** at least one `VIDIOC_ENUM_FMT` result. Logical indices `0` and `1` (the existing `--left-cam` / `--right-cam` semantics) map onto that filtered, numerically-sorted list. `logConnectedDevices()` prints the mapping along with the stable `/dev/v4l/by-id/` alias so a fixed path can be pinned with `--left-dev` / `--right-dev` when USB enumeration order is unreliable.

---

## 4. Pixel Format Negotiation

Formats are tried in this order; the first one the device advertises wins. All three yield a `CV_8UC1` Y-plane without decoding:

| Priority | V4L2 fourcc | Layout | Copy strategy in `grabRawFrame` |
| :---: | :--- | :--- | :--- |
| 1 | `GREY` | 8-bit mono | Strided `copyTo` (same as MF `L8`) |
| 2 | `NV12` | Y plane first, then interleaved UV | Same strided `copyTo` over the leading `bytesperline × height` bytes |
| 3 | `YUYV` | `Y0 U Y1 V …` | Wrap as `CV_8UC2`, `cv::extractChannel(…, 0)` |

`MJPEG` is deliberately unsupported (decode cost on the hot path). The driver requests 1280 × 800 and reads back the negotiated size; a mismatch is logged because `FrameSet::preallocate(1280, 800)` upstream is fixed and `copyTo` would silently reallocate on every frame.

Frame rate is set to the shortest interval the device enumerates for the chosen format/size via `VIDIOC_S_PARM`.

---

## 5. Exposure and Gain

UVC exposes exposure as `V4L2_CID_EXPOSURE_ABSOLUTE` in **100 µs units**. At init the driver:

1. Sets `V4L2_CID_EXPOSURE_AUTO = V4L2_EXPOSURE_MANUAL`
2. Sets `V4L2_CID_EXPOSURE_AUTO_PRIORITY = 0` so the driver never lowers frame rate to reach the exposure
3. Queries the `EXPOSURE_ABSOLUTE` range and applies **10 000 µs (value 100)** — the 10 ms window from refactor 02 that captures three 300 Hz pulses
4. Disables `V4L2_CID_AUTOGAIN` if present

`setHardwareExposure(us)` converts with `round(us / 100)` clamped to the queried range. The `--live` viewer's presets map to 5, 10, 20, 50, 100.

---

## 6. Streaming and the Shutdown Handshake

Four `V4L2_MEMORY_MMAP` buffers are requested, mapped, queued, then `VIDIOC_STREAMON`. A shallow queue keeps latency low; the `AtomicRingBuffer` upstream absorbs jitter.

`grabRawFrame` is: `poll(fd, POLLIN, 100 ms)` → `VIDIOC_DQBUF` → strided copy → record `v4l2_buffer.timestamp` → `VIDIOC_QBUF`. It never drains to "latest" — shots are two frames long and every frame matters.

`ThreadManager::stop()` calls `cameraSystem->shutdown()` from the main thread while the producer may be blocked in `poll`. The ordering in `V4L2Driver::shutdown()` makes that safe:

```
main thread                          producer thread
-----------                          ---------------
streaming_ = false
VIDIOC_STREAMOFF  ──────────────►    poll() returns POLLERR / DQBUF fails
lock(grabMutex_)  ◄──────────────    grabRawFrame returns false, mutex released
munmap buffers, REQBUFS(0), close
```

If `VIDIOC_STREAMON` fails with `ENOSPC`, the driver logs the specific cause: USB isochronous bandwidth is exhausted (two 1 MP cameras on one USB 2.0 host controller). Move one camera to a different controller / USB 3 port, or load `uvcvideo` with `quirks=0x80`.

---

## 7. Frame Timestamps

`FrameSet::timestamp` was never populated by the producer, so every `metadata.json` frame recorded `0`. `ICameraNode` and `IUsbVideoDriver` gained a defaulted `getLastFrameTimestampUs()`; `HardwareSyncedCameraSystem` stamps each `FrameSet` with the left camera's value. On Linux this is the kernel monotonic capture time; on Windows it is the Media Foundation sample time (100 ns → µs). Replays now carry real inter-frame Δt for review.

---

## 8. Linux Permissions

| Device | Group (Arch `50-udev-default.rules`) | `uaccess` ACL for seat user |
| :--- | :--- | :---: |
| `/dev/video*` | `video` | Yes — works out of the box when logged in locally |
| `/dev/ttyACM*` (strobe controller) | `uucp` | No |

One-time: `sudo usermod -aG uucp $USER`, then log out and back in.
