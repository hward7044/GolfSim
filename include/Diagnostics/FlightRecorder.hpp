#pragma once
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Camera/FrameSet.hpp"
#include "Math/ITriggerDetector.hpp"
#include "Math/IComputerVision.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/LaunchData.hpp"
#include "Math/Units.hpp"

struct RecordedFrame {
    uint64_t timestamp = 0;
    cv::Mat leftFrame;
    cv::Mat rightFrame;
    TriggerDiagnostics triggerDiag;
    VisionDiagnostics leftVisionDiag;
    VisionDiagnostics rightVisionDiag;
    std::vector<Ball3D> triangulatedBalls;
};

class FlightRecorder {
private:
    std::string outputDirectory;
    void enforceLimit();
public:
    FlightRecorder(const std::string& outDir = "build/replays");

    void saveSession(
        const std::vector<RecordedFrame>& frames,
        const LaunchData<Degrees, MilesPerHour>& launchData
    );
};
