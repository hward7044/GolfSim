#pragma once
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/core.hpp>
#include "Camera/FrameSet.hpp"
#include "Math/ISpatialSolver.hpp"
#include "Math/LaunchData.hpp"
#include "Math/Units.hpp"
#include <nlohmann/json.hpp>

struct RecordedFrame {
    uint64_t timestamp = 0;
    cv::Mat leftFrame;
    cv::Mat rightFrame;
    nlohmann::json triggerDiag;
    nlohmann::json leftVisionDiag;
    nlohmann::json rightVisionDiag;
    std::vector<Ball3D> triangulatedBalls;
};

class FlightRecorder {
private:
    struct SaveTask {
        std::vector<RecordedFrame> frames;
        LaunchData<Degrees, MilesPerHour> launchData;
    };

    std::string outputDirectory;
    
    std::queue<SaveTask> taskQueue;
    std::mutex           queueMutex;
    std::condition_variable cvQueue;
    std::thread          workerThread;
    std::atomic<bool>    stopWorker;

    void workerLoop();
    void enforceLimit();
    void processSaveTask(const SaveTask& task);
public:
    FlightRecorder(const std::string& outDir = "build/replays");
    ~FlightRecorder();

    // Prevent copying
    FlightRecorder(const FlightRecorder&) = delete;
    FlightRecorder& operator=(const FlightRecorder&) = delete;

    void saveSession(
        const std::vector<RecordedFrame>& frames,
        const LaunchData<Degrees, MilesPerHour>& launchData
    );
};
