#pragma once
#include <string>
#include <vector>
#include "FrameSet.hpp"
class FlightRecorder {
private:
    std::string outputDirectory;
public:
    void saveSession(const std::vector<FrameSet>& frames);
};
