#include "HAL/V4L2Driver.hpp"
#ifdef __linux__

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <regex>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

// =============================================================================
// Constants
// =============================================================================
static constexpr uint32_t REQUESTED_WIDTH   = 1280;
static constexpr uint32_t REQUESTED_HEIGHT  = 800;
static constexpr uint32_t MMAP_BUFFER_COUNT = 4;      // Shallow queue: low latency, ring buffer absorbs jitter
static constexpr int      GRAB_POLL_TIMEOUT_MS = 100; // Bounded so shutdown() can interrupt a stalled grab
static constexpr int      DEFAULT_EXPOSURE_US = 10000; // 10 ms: captures 3 pulses at 300 Hz (refactor 02)

// Pixel formats we can turn into a CV_8UC1 Y-plane without decoding, in preference order.
static constexpr uint32_t PREFERRED_FORMATS[] = {
    V4L2_PIX_FMT_GREY,   // Native 8-bit mono — ideal
    V4L2_PIX_FMT_NV12,   // Y plane is contiguous at the start of the buffer
    V4L2_PIX_FMT_YUYV,   // Y interleaved every other byte
};

// =============================================================================
// Helpers
// =============================================================================

// ioctl wrapper that retries when interrupted by a signal.
static int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

static std::string fourccToString(uint32_t fourcc) {
    char s[5] = {
        static_cast<char>(fourcc & 0xFF),
        static_cast<char>((fourcc >> 8) & 0xFF),
        static_cast<char>((fourcc >> 16) & 0xFF),
        static_cast<char>((fourcc >> 24) & 0xFF),
        '\0'
    };
    return std::string(s);
}

static bool setControl(int fd, uint32_t id, int32_t value) {
    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;
    return xioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0;
}

// Returns true if the control exists and fills min/max; false if unsupported.
static bool queryControlRange(int fd, uint32_t id, int32_t& minOut, int32_t& maxOut, int32_t& defOut) {
    v4l2_queryctrl q{};
    q.id = id;
    if (xioctl(fd, VIDIOC_QUERYCTRL, &q) != 0) return false;
    if (q.flags & V4L2_CTRL_FLAG_DISABLED) return false;
    minOut = q.minimum;
    maxOut = q.maximum;
    defOut = q.default_value;
    return true;
}

// Numeric sort of /dev/videoN so index mapping is stable (video10 after video2).
static std::vector<fs::path> listVideoNodes() {
    std::vector<fs::path> nodes;
    static const std::regex pattern("^video([0-9]+)$");
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator("/dev", ec)) {
        std::smatch m;
        std::string name = entry.path().filename().string();
        if (std::regex_match(name, m, pattern)) {
            nodes.push_back(entry.path());
        }
    }
    std::sort(nodes.begin(), nodes.end(), [](const fs::path& a, const fs::path& b) {
        auto num = [](const fs::path& p) {
            return std::stoi(p.filename().string().substr(5));
        };
        return num(a) < num(b);
    });
    return nodes;
}

// Look up the stable /dev/v4l/by-id/ symlink that points at this node, if any.
static std::string findByIdAlias(const fs::path& node) {
    std::error_code ec;
    fs::path byId("/dev/v4l/by-id");
    if (!fs::exists(byId, ec)) return "";
    for (const auto& entry : fs::directory_iterator(byId, ec)) {
        fs::path target = fs::read_symlink(entry.path(), ec);
        if (ec) continue;
        fs::path resolved = fs::weakly_canonical(byId / target, ec);
        if (!ec && resolved == node) {
            return entry.path().string();
        }
    }
    return "";
}

// A node is a usable camera if it advertises video capture AND at least one
// capture format. The uvcvideo metadata node (V4L2_CAP_META_CAPTURE) fails both.
static bool isCaptureNode(int fd, v4l2_capability* capOut = nullptr) {
    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) != 0) return false;
    uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) return false;
    if (!(caps & V4L2_CAP_STREAMING)) return false;

    v4l2_fmtdesc fmt{};
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_ENUM_FMT, &fmt) != 0) return false;

    if (capOut) *capOut = cap;
    return true;
}

// =============================================================================
// Static enumeration
// =============================================================================

std::vector<std::string> V4L2Driver::enumerateCaptureDevices() {
    std::vector<std::string> devices;
    for (const auto& node : listVideoNodes()) {
        int fd = ::open(node.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;
        if (isCaptureNode(fd)) {
            devices.push_back(node.string());
        }
        ::close(fd);
    }
    return devices;
}

void V4L2Driver::logConnectedDevices() {
    auto nodes = listVideoNodes();
    if (nodes.empty()) {
        spdlog::warn("[V4L2Driver] No /dev/video* nodes present.");
        return;
    }

    spdlog::info("[V4L2Driver] Enumerating V4L2 devices ({} nodes):", nodes.size());
    uint32_t logicalIndex = 0;
    for (const auto& node : nodes) {
        int fd = ::open(node.c_str(), O_RDWR | O_NONBLOCK);
        if (fd < 0) {
            spdlog::warn("[V4L2Driver]   {}  (cannot open: {})", node.string(), strerror(errno));
            continue;
        }
        v4l2_capability cap{};
        if (isCaptureNode(fd, &cap)) {
            std::string alias = findByIdAlias(node);
            spdlog::info("[V4L2Driver]   [{}] {}  card='{}' driver='{}' bus='{}'{}",
                         logicalIndex++, node.string(),
                         reinterpret_cast<const char*>(cap.card),
                         reinterpret_cast<const char*>(cap.driver),
                         reinterpret_cast<const char*>(cap.bus_info),
                         alias.empty() ? "" : "  by-id=" + alias);

            v4l2_fmtdesc fmt{};
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            for (fmt.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0; ++fmt.index) {
                spdlog::info("[V4L2Driver]        fmt {} ({})",
                             fourccToString(fmt.pixelformat),
                             reinterpret_cast<const char*>(fmt.description));
            }
        } else {
            spdlog::debug("[V4L2Driver]   {}  (not a video capture node — skipped)", node.string());
        }
        ::close(fd);
    }
}

// =============================================================================
// Construction / Destruction
// =============================================================================

V4L2Driver::V4L2Driver(uint32_t logicalIndex) {
    auto devices = enumerateCaptureDevices();
    if (logicalIndex < devices.size()) {
        devicePath_ = devices[logicalIndex];
    } else {
        spdlog::warn("[V4L2Driver] Logical camera index {} out of range ({} capture devices found).",
                     logicalIndex, devices.size());
    }
}

V4L2Driver::V4L2Driver(std::string devicePath)
    : devicePath_(std::move(devicePath)) {}

V4L2Driver::~V4L2Driver() {
    shutdown();
}

// =============================================================================
// Lifecycle: initialize / shutdown
// =============================================================================

bool V4L2Driver::initialize() {
    if (initialized_) return true;  // Idempotent

    if (devicePath_.empty()) {
        spdlog::error("[V4L2Driver] No device path resolved; cannot initialize.");
        return false;
    }

    if (!openDevice()      ||
        !negotiateFormat() ||
        !selectMaxFrameRate()) {
        shutdown();
        return false;
    }

    configureControls();  // Best-effort; never fatal

    if (!setupBuffers() || !startStreaming()) {
        shutdown();
        return false;
    }

    initialized_ = true;
    spdlog::info("[V4L2Driver] {} ready: {}x{} {} stride={}",
                 devicePath_, width_, height_, fourccToString(pixFmt_), stride_);
    return true;
}

void V4L2Driver::shutdown() {
    // 1. Flip the flag so a grab that is about to start bails out immediately.
    streaming_ = false;

    // 2. STREAMOFF from this thread wakes any poll()/DQBUF blocked in the
    //    producer thread — this is what ThreadManager::stop() relies on.
    if (fd_ >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }

    // 3. Wait for any in-flight grab to release the buffers before we unmap.
    std::lock_guard<std::mutex> lock(grabMutex_);
    releaseBuffers();

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    initialized_ = false;
}

// =============================================================================
// Internal: open + capability check
// =============================================================================

bool V4L2Driver::openDevice() {
    // Blocking fd: we bound waits with poll() rather than O_NONBLOCK spinning.
    fd_ = ::open(devicePath_.c_str(), O_RDWR);
    if (fd_ < 0) {
        spdlog::error("[V4L2Driver] Failed to open {}: {}", devicePath_, strerror(errno));
        return false;
    }

    v4l2_capability cap{};
    if (!isCaptureNode(fd_, &cap)) {
        spdlog::error("[V4L2Driver] {} is not a streaming video capture device.", devicePath_);
        return false;
    }
    spdlog::info("[V4L2Driver] Opened {} (card='{}', driver='{}')", devicePath_,
                 reinterpret_cast<const char*>(cap.card),
                 reinterpret_cast<const char*>(cap.driver));
    return true;
}

// =============================================================================
// Internal: pixel format negotiation
// =============================================================================

bool V4L2Driver::negotiateFormat() {
    // Collect what the device offers
    std::vector<uint32_t> offered;
    v4l2_fmtdesc desc{};
    desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (desc.index = 0; xioctl(fd_, VIDIOC_ENUM_FMT, &desc) == 0; ++desc.index) {
        offered.push_back(desc.pixelformat);
    }

    uint32_t chosen = 0;
    for (uint32_t pref : PREFERRED_FORMATS) {
        if (std::find(offered.begin(), offered.end(), pref) != offered.end()) {
            chosen = pref;
            break;
        }
    }

    if (chosen == 0) {
        std::string list;
        for (uint32_t f : offered) list += fourccToString(f) + " ";
        spdlog::error("[V4L2Driver] {} offers no GREY/NV12/YUYV format. Available: [{}]",
                      devicePath_, list);
        return false;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = REQUESTED_WIDTH;
    fmt.fmt.pix.height      = REQUESTED_HEIGHT;
    fmt.fmt.pix.pixelformat = chosen;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) != 0) {
        spdlog::error("[V4L2Driver] VIDIOC_S_FMT failed on {}: {}", devicePath_, strerror(errno));
        return false;
    }

    // The driver may adjust any of these — always read back.
    width_  = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    stride_ = fmt.fmt.pix.bytesperline;
    pixFmt_ = fmt.fmt.pix.pixelformat;

    if (pixFmt_ != chosen) {
        spdlog::error("[V4L2Driver] Driver substituted format {} for requested {}.",
                      fourccToString(pixFmt_), fourccToString(chosen));
        return false;
    }
    if (width_ != REQUESTED_WIDTH || height_ != REQUESTED_HEIGHT) {
        // FrameSet::preallocate(1280, 800) is hardcoded upstream; a mismatch makes
        // copyTo() reallocate on every frame and breaks the zero-alloc contract.
        spdlog::warn("[V4L2Driver] Requested {}x{} but driver negotiated {}x{}. "
                     "Upstream buffers are sized for {}x{}.",
                     REQUESTED_WIDTH, REQUESTED_HEIGHT, width_, height_,
                     REQUESTED_WIDTH, REQUESTED_HEIGHT);
    }
    if (stride_ == 0) {
        stride_ = (pixFmt_ == V4L2_PIX_FMT_YUYV) ? width_ * 2 : width_;
    }

    spdlog::info("[V4L2Driver] Negotiated {} {}x{} (stride {})",
                 fourccToString(pixFmt_), width_, height_, stride_);
    return true;
}

// =============================================================================
// Internal: frame rate — pick the shortest interval the device advertises
// =============================================================================

bool V4L2Driver::selectMaxFrameRate() {
    v4l2_frmivalenum ival{};
    ival.pixel_format = pixFmt_;
    ival.width  = width_;
    ival.height = height_;

    double bestInterval = 0.0;  // seconds; 0 = none found
    for (ival.index = 0; xioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &ival) == 0; ++ival.index) {
        double seconds = 0.0;
        if (ival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            seconds = static_cast<double>(ival.discrete.numerator) / ival.discrete.denominator;
        } else {
            // Stepwise/continuous: the minimum interval is the fastest rate
            seconds = static_cast<double>(ival.stepwise.min.numerator) / ival.stepwise.min.denominator;
        }
        if (seconds > 0.0 && (bestInterval == 0.0 || seconds < bestInterval)) {
            bestInterval = seconds;
            if (ival.type != V4L2_FRMIVAL_TYPE_DISCRETE) break;
        }
    }

    if (bestInterval == 0.0) {
        spdlog::warn("[V4L2Driver] No frame intervals advertised; leaving driver default rate.");
        return true;  // Not fatal — stream at whatever the device defaults to
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_G_PARM, &parm) != 0) {
        spdlog::warn("[V4L2Driver] VIDIOC_G_PARM failed: {}", strerror(errno));
        return true;
    }
    if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
        spdlog::info("[V4L2Driver] Device does not support setting frame rate; using default.");
        return true;
    }

    // Express the chosen interval with the same fraction the driver enumerated
    v4l2_frmivalenum pick{};
    pick.pixel_format = pixFmt_;
    pick.width  = width_;
    pick.height = height_;
    uint32_t num = 1, den = static_cast<uint32_t>(std::lround(1.0 / bestInterval));
    for (pick.index = 0; xioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &pick) == 0; ++pick.index) {
        if (pick.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            double s = static_cast<double>(pick.discrete.numerator) / pick.discrete.denominator;
            if (std::abs(s - bestInterval) < 1e-9) {
                num = pick.discrete.numerator;
                den = pick.discrete.denominator;
                break;
            }
        }
    }

    parm.parm.capture.timeperframe.numerator   = num;
    parm.parm.capture.timeperframe.denominator = den;
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) != 0) {
        spdlog::warn("[V4L2Driver] VIDIOC_S_PARM failed: {}", strerror(errno));
        return true;
    }

    double achieved = static_cast<double>(parm.parm.capture.timeperframe.denominator) /
                      parm.parm.capture.timeperframe.numerator;
    spdlog::info("[V4L2Driver] Frame rate set to {:.1f} fps", achieved);
    return true;
}

// =============================================================================
// Internal: manual exposure / gain — best effort, never fatal
// =============================================================================

void V4L2Driver::configureControls() {
    // Manual exposure so the strobe pulses, not ambient light, define the image.
    if (!setControl(fd_, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL)) {
        spdlog::warn("[V4L2Driver] Could not set EXPOSURE_AUTO=MANUAL: {}", strerror(errno));
    }
    // Stop the driver dropping the frame rate to satisfy a long exposure.
    if (!setControl(fd_, V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0)) {
        spdlog::debug("[V4L2Driver] EXPOSURE_AUTO_PRIORITY not supported ({})", strerror(errno));
    }

    int32_t def = 0;
    exposureSupported_ = queryControlRange(fd_, V4L2_CID_EXPOSURE_ABSOLUTE,
                                           exposureMin_, exposureMax_, def);
    if (exposureSupported_) {
        spdlog::info("[V4L2Driver] Exposure range: {}..{} (x100us), default {}",
                     exposureMin_, exposureMax_, def);
        setHardwareExposure(DEFAULT_EXPOSURE_US);
    } else {
        spdlog::warn("[V4L2Driver] EXPOSURE_ABSOLUTE control not available; exposure left at driver default.");
    }

    int32_t gMin, gMax, gDef;
    if (queryControlRange(fd_, V4L2_CID_AUTOGAIN, gMin, gMax, gDef)) {
        setControl(fd_, V4L2_CID_AUTOGAIN, 0);
    }
}

// =============================================================================
// Internal: MMAP buffer setup / teardown / streaming
// =============================================================================

bool V4L2Driver::setupBuffers() {
    v4l2_requestbuffers req{};
    req.count  = MMAP_BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) != 0) {
        spdlog::error("[V4L2Driver] VIDIOC_REQBUFS failed: {}", strerror(errno));
        return false;
    }
    if (req.count < 2) {
        spdlog::error("[V4L2Driver] Driver granted only {} buffer(s); need at least 2.", req.count);
        return false;
    }

    buffers_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) != 0) {
            spdlog::error("[V4L2Driver] VIDIOC_QUERYBUF[{}] failed: {}", i, strerror(errno));
            return false;
        }
        void* start = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (start == MAP_FAILED) {
            spdlog::error("[V4L2Driver] mmap of buffer {} failed: {}", i, strerror(errno));
            buffers_[i].start = nullptr;
            return false;
        }
        buffers_[i].start  = start;
        buffers_[i].length = buf.length;
    }

    // Hand every buffer to the driver before STREAMON
    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buf) != 0) {
            spdlog::error("[V4L2Driver] Initial VIDIOC_QBUF[{}] failed: {}", i, strerror(errno));
            return false;
        }
    }
    return true;
}

void V4L2Driver::releaseBuffers() {
    for (auto& b : buffers_) {
        if (b.start && b.start != MAP_FAILED) {
            ::munmap(b.start, b.length);
        }
        b.start = nullptr;
        b.length = 0;
    }
    buffers_.clear();

    if (fd_ >= 0) {
        // Release the driver-side allocation so the device can be reopened cleanly
        v4l2_requestbuffers req{};
        req.count  = 0;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd_, VIDIOC_REQBUFS, &req);
    }
}

bool V4L2Driver::startStreaming() {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) != 0) {
        int err = errno;
        spdlog::error("[V4L2Driver] VIDIOC_STREAMON failed on {}: {}", devicePath_, strerror(err));
        if (err == ENOSPC) {
            spdlog::error("[V4L2Driver] ENOSPC = USB bandwidth exhausted. Put the two cameras on "
                          "separate host controllers / USB 3 ports, or load uvcvideo with quirks=0x80.");
        }
        return false;
    }
    streaming_ = true;
    return true;
}

// =============================================================================
// HOT PATH: grabRawFrame — one poll, one DQBUF, one strided copy, one QBUF
// =============================================================================

bool V4L2Driver::grabRawFrame(cv::Mat& destination) {
    std::lock_guard<std::mutex> lock(grabMutex_);
    if (!streaming_ || fd_ < 0) return false;

    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    int pr = ::poll(&pfd, 1, GRAB_POLL_TIMEOUT_MS);
    if (pr <= 0) return false;                         // timeout or signal — caller retries
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        return false;                                  // STREAMOFF from shutdown(), or device gone
    }

    v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) != 0) {
        return false;                                  // EAGAIN / EINVAL after STREAMOFF
    }

    bool ok = false;
    if (buf.index < buffers_.size() && buffers_[buf.index].start) {
        const uint8_t* data = static_cast<const uint8_t*>(buffers_[buf.index].start);
        const size_t requiredBytes = static_cast<size_t>(stride_) * height_;

        if (!(buf.flags & V4L2_BUF_FLAG_ERROR) && buf.bytesused >= requiredBytes) {
            if (pixFmt_ == V4L2_PIX_FMT_YUYV) {
                // Y is byte 0 of every 2-byte pixel; extractChannel writes straight
                // into the preallocated destination.
                const cv::Mat wrapper(static_cast<int>(height_), static_cast<int>(width_),
                                      CV_8UC2, const_cast<uint8_t*>(data), stride_);
                cv::extractChannel(wrapper, destination, 0);
            } else {
                // GREY, or the leading Y plane of NV12 — a plain strided copy.
                const cv::Mat wrapper(static_cast<int>(height_), static_cast<int>(width_),
                                      CV_8UC1, const_cast<uint8_t*>(data), stride_);
                wrapper.copyTo(destination);
            }
            lastTimestampUs_ = static_cast<uint64_t>(buf.timestamp.tv_sec) * 1000000ULL +
                               static_cast<uint64_t>(buf.timestamp.tv_usec);
            ok = true;
        }
    }

    // Always give the buffer back, even on a bad frame
    xioctl(fd_, VIDIOC_QBUF, &buf);
    return ok;
}

// =============================================================================
// setHardwareExposure — mode transitions only, not on the hot path
// =============================================================================

void V4L2Driver::setHardwareExposure(int microseconds) {
    if (fd_ < 0 || !exposureSupported_ || microseconds < 0) return;

    // UVC exposure_time_absolute is in 100 us units
    int32_t units = static_cast<int32_t>(std::lround(microseconds / 100.0));
    units = std::clamp(units, exposureMin_, exposureMax_);

    if (setControl(fd_, V4L2_CID_EXPOSURE_ABSOLUTE, units)) {
        spdlog::debug("[V4L2Driver] Exposure set to {} us ({} x100us)", units * 100, units);
    } else {
        spdlog::warn("[V4L2Driver] Failed to set exposure {} us: {}", microseconds, strerror(errno));
    }
}

void V4L2Driver::injectImmediateRegisterWrite(uint16_t reg, uint8_t value) {
    // Disabled: the standard UVC firmware on this Arducam OV9281 bridge exposes no
    // Extension Unit for I2C pass-through (see scratch/dump_xu.py), so there is no
    // UVCIOC_CTRL_QUERY target. Mirrors the MediaFoundationDriver behaviour.
    (void)reg;
    (void)value;
}

#endif
