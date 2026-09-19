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
    enum class TaskType {
        SHOT,
        STREAM
    };

    struct SaveTask {
        TaskType type = TaskType::SHOT;
        std::string sessionTimestamp;
        std::vector<RecordedFrame> frames;
        LaunchData<Degrees, MilesPerHour> launchData;
    };

    std::string outputDirectory;
    nlohmann::json sessionInfo_;   // Applied camera/detector config, written into every metadata.json
    
    std::queue<SaveTask> taskQueue;
    std::mutex           queueMutex;
    std::condition_variable cvQueue;
    std::thread          workerThread;
    std::atomic<bool>    stopWorker;

    void workerLoop();
    void enforceLimit();
    void processSaveTask(const SaveTask& task);
    void processStreamTask(const SaveTask& task);
public:
    /// Draw the diagnostic overlays for one camera frame: trigger state, dot
    /// clusters (accepted green / rejected red), individual dots, 3D position.
    static cv::Mat annotateFrame(const cv::Mat& gray,
                                 const nlohmann::json& triggerDiag,
                                 const nlohmann::json& visionDiag,
                                 const std::vector<Ball3D>& balls3D,
                                 bool isLeft);

    FlightRecorder(const std::string& outDir = "build/replays");
    ~FlightRecorder();

    // Prevent copying
    FlightRecorder(const FlightRecorder&) = delete;
    FlightRecorder& operator=(const FlightRecorder&) = delete;

    /// Recorded into every metadata.json under "session" (e.g. the applied
    /// CameraConfig), so a replay says what the hardware was set to.
    void setSessionInfo(nlohmann::json info) { sessionInfo_ = std::move(info); }

    void saveSession(
        const std::vector<RecordedFrame>& frames,
        const LaunchData<Degrees, MilesPerHour>& launchData
    );

    void saveStreamSession(
        const std::vector<RecordedFrame>& frames
    );
};
