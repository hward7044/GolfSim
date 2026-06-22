#pragma once
#include <string>
#include <ranges>
#include "Camera/FrameSet.hpp"
class FlightRecorder {
private:
    std::string outputDirectory;
public:
    // T.3 — express containers & ranges
    // template<typename FrameRange>
    //   requires std::ranges::range<FrameRange>
    // Accepts span, deque, ring-buffer view, etc.
    template<typename FrameRange>
        requires std::ranges::range<FrameRange>
    void saveSession(FrameRange&& frames) {
        // Stub: save frames to outputDirectory
    }
};
