// CameraBench — measures what the camera drivers actually deliver, with
// nothing downstream of them. The live viewer and stream mode both run the
// detector, diagnostics and display in the same loop, so their frame counters
// report the consumer's speed, not the cameras'. This tool answers one
// question: at a requested media type, how many frames per second reach
// grabRawFrame(), and how are they spaced?
//
//   CameraBench [--fps N] [--frames N] [--exposure-us N] [--gain N]
//               [--left-cam I] [--right-cam I] [--mode single|seq|par|all]
//
// Modes:
//   single  each camera opened on its own, tight grabRawFrame() loop
//   seq     both open; left then right in one thread (the production pattern
//           in HardwareSyncedCameraSystem::captureSynchronizedFrames)
//   par     both open; one thread per camera
//
// For every run it reports the wall-clock rate, the per-read latency, and the
// spacing of the driver's own sample timestamps: the median gap is the rate
// the camera is producing at, and gaps well over one period are frames the
// USB/driver path dropped before we saw them.

#include "Camera/CameraConfig.hpp"
#ifdef _WIN32
#include "HAL/MediaFoundationDriver.hpp"
using PlatformCameraDriver = MediaFoundationDriver;
#else
#include "HAL/V4L2Driver.hpp"
using PlatformCameraDriver = V4L2Driver;
#endif

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct RunStats {
    std::string label;
    int    frames        = 0;
    double wallSeconds   = 0.0;
    double wallFps       = 0.0;
    double readMeanUs    = 0.0;   // time spent inside grabRawFrame()
    double readMaxUs     = 0.0;
    double tsMedianUs    = 0.0;   // spacing of the driver's sample timestamps
    double tsFps         = 0.0;
    int    tsGaps        = 0;     // timestamp gaps > 1.5x the median (dropped frames)
    int    failures      = 0;
};

struct Sample { double readUs; uint64_t tsUs; };

// Tight capture loop on one driver.
RunStats capture(PlatformCameraDriver& drv, cv::Mat& dst, int frames, int warmup, const std::string& label) {
    RunStats s; s.label = label;
    std::vector<Sample> samples; samples.reserve(frames);
    for (int i = 0; i < warmup; ++i) drv.grabRawFrame(dst);

    const auto t0 = Clock::now();
    for (int i = 0; i < frames; ++i) {
        const auto a = Clock::now();
        const bool ok = drv.grabRawFrame(dst);
        const auto b = Clock::now();
        if (!ok) { ++s.failures; continue; }
        samples.push_back({std::chrono::duration<double, std::micro>(b - a).count(),
                           drv.getLastFrameTimestampUs()});
    }
    s.wallSeconds = std::chrono::duration<double>(Clock::now() - t0).count();
    s.frames  = static_cast<int>(samples.size());
    s.wallFps = s.frames / s.wallSeconds;

    if (samples.empty()) return s;
    double sum = 0.0;
    for (const auto& x : samples) { sum += x.readUs; s.readMaxUs = std::max(s.readMaxUs, x.readUs); }
    s.readMeanUs = sum / samples.size();

    std::vector<double> gaps;
    for (size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].tsUs > samples[i - 1].tsUs)
            gaps.push_back(static_cast<double>(samples[i].tsUs - samples[i - 1].tsUs));
    }
    if (!gaps.empty()) {
        std::vector<double> sorted = gaps;
        std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
        s.tsMedianUs = sorted[sorted.size() / 2];
        s.tsFps = s.tsMedianUs > 0 ? 1e6 / s.tsMedianUs : 0.0;
        for (double g : gaps) if (g > 1.5 * s.tsMedianUs) ++s.tsGaps;
    }
    return s;
}

void print(const RunStats& s) {
    std::printf("  %-22s %4d frames in %6.2fs  wall %6.1f fps | read mean %6.0f us max %6.0f us"
                " | ts median %6.0f us (%5.1f fps), %3d gaps, %d failures\n",
                s.label.c_str(), s.frames, s.wallSeconds, s.wallFps, s.readMeanUs, s.readMaxUs,
                s.tsMedianUs, s.tsFps, s.tsGaps, s.failures);
}

std::unique_ptr<PlatformCameraDriver> open(int index, const CameraConfig& cfg) {
    auto d = std::make_unique<PlatformCameraDriver>(static_cast<uint32_t>(index), cfg);
    if (!d->initialize()) {
        std::fprintf(stderr, "camera %d failed to initialise\n", index);
        return nullptr;
    }
    return d;
}

} // namespace

int main(int argc, char* argv[]) {
    CameraConfig cfg;
    int frames = 500, leftIdx = 0, rightIdx = 1;
    std::string mode = "all";
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string a = argv[i];
        if      (a == "--fps")         cfg.targetFps  = std::atoi(argv[i + 1]);
        else if (a == "--frames")      frames         = std::atoi(argv[i + 1]);
        else if (a == "--exposure-us") cfg.exposureUs = std::atoi(argv[i + 1]);
        else if (a == "--gain")        cfg.gain       = std::atoi(argv[i + 1]);
        else if (a == "--left-cam")    leftIdx        = std::atoi(argv[i + 1]);
        else if (a == "--right-cam")   rightIdx       = std::atoi(argv[i + 1]);
        else if (a == "--mode")        mode           = argv[i + 1];
    }
    spdlog::set_level(spdlog::level::warn);   // the drivers log every media type at info
    const int warmup = 30;

    std::printf("CameraBench: requesting %d fps, exposure %d us, gain %d, %d frames per run (+%d warm-up)\n",
                cfg.targetFps, cfg.exposureUs, cfg.gain, frames, warmup);

    if (mode == "single" || mode == "all") {
        std::printf("[single] one camera open at a time\n");
        for (int idx : {leftIdx, rightIdx}) {
            auto d = open(idx, cfg);
            if (!d) continue;
            cv::Mat dst(static_cast<int>(d->getFrameHeight()), static_cast<int>(d->getFrameWidth()), CV_8UC1);
            std::printf("  camera %d negotiated %.1f fps, exposure %d us\n",
                        idx, d->getNegotiatedFps(), d->getHardwareExposureUs());
            print(capture(*d, dst, frames, warmup, "cam " + std::to_string(idx) + " alone"));
            d->shutdown();
        }
    }

    if (mode == "seq" || mode == "par" || mode == "all") {
        auto L = open(leftIdx, cfg);
        auto R = open(rightIdx, cfg);
        if (!L || !R) return 1;
        cv::Mat dL(static_cast<int>(L->getFrameHeight()), static_cast<int>(L->getFrameWidth()), CV_8UC1);
        cv::Mat dR(static_cast<int>(R->getFrameHeight()), static_cast<int>(R->getFrameWidth()), CV_8UC1);
        std::printf("[pair] both open: left %d @ %.1f fps, right %d @ %.1f fps\n",
                    leftIdx, L->getNegotiatedFps(), rightIdx, R->getNegotiatedFps());

        if (mode == "seq" || mode == "all") {
            // Production pattern: left blocks for its next frame, then right.
            std::vector<Sample> tl, tr;
            int failures = 0;
            for (int i = 0; i < warmup; ++i) { L->grabRawFrame(dL); R->grabRawFrame(dR); }
            const auto t0 = Clock::now();
            for (int i = 0; i < frames; ++i) {
                const auto a = Clock::now();
                const bool okL = L->grabRawFrame(dL);
                const auto b = Clock::now();
                const bool okR = R->grabRawFrame(dR);
                const auto c = Clock::now();
                if (!okL || !okR) { ++failures; continue; }
                tl.push_back({std::chrono::duration<double, std::micro>(b - a).count(), L->getLastFrameTimestampUs()});
                tr.push_back({std::chrono::duration<double, std::micro>(c - b).count(), R->getLastFrameTimestampUs()});
            }
            const double wall = std::chrono::duration<double>(Clock::now() - t0).count();
            double sl = 0, sr = 0, ml = 0, mr = 0;
            for (const auto& x : tl) { sl += x.readUs; ml = std::max(ml, x.readUs); }
            for (const auto& x : tr) { sr += x.readUs; mr = std::max(mr, x.readUs); }
            std::printf("  %-22s %4zu pairs  in %6.2fs  wall %6.1f pairs/s | L read mean %6.0f us max %6.0f us"
                        " | R read mean %6.0f us max %6.0f us | %d failures\n",
                        "seq L then R", tl.size(), wall, tl.size() / wall,
                        tl.empty() ? 0.0 : sl / tl.size(), ml, tr.empty() ? 0.0 : sr / tr.size(), mr, failures);
            // Left-right skew: how far apart the two sample timestamps of a pair are.
            double skewSum = 0, skewMax = 0;
            for (size_t i = 0; i < tl.size() && i < tr.size(); ++i) {
                const double d = std::fabs(static_cast<double>(tl[i].tsUs) - static_cast<double>(tr[i].tsUs));
                skewSum += d; skewMax = std::max(skewMax, d);
            }
            if (!tl.empty())
                std::printf("  %-22s L-R timestamp skew mean %6.0f us max %6.0f us\n", "", skewSum / tl.size(), skewMax);
        }

        if (mode == "par" || mode == "all") {
            RunStats sL, sR;
            std::thread a([&] { sL = capture(*L, dL, frames, warmup, "par left"); });
            std::thread b([&] { sR = capture(*R, dR, frames, warmup, "par right"); });
            a.join(); b.join();
            print(sL); print(sR);
        }
        L->shutdown(); R->shutdown();
    }
    return 0;
}
