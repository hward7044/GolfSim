#include "Diagnostics/FlightRecorder.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

FlightRecorder::FlightRecorder(const std::string& outDir)
    : outputDirectory(outDir), stopWorker(false) {
    try {
        fs::create_directories(outputDirectory);
    } catch (const std::exception& e) {
        spdlog::error("[FlightRecorder] Failed to create output directory {}: {}", outputDirectory, e.what());
    }
    workerThread = std::thread(&FlightRecorder::workerLoop, this);
}

FlightRecorder::~FlightRecorder() {
    stopWorker = true;
    cvQueue.notify_all();
    if (workerThread.joinable()) {
        workerThread.join();
    }
}

void FlightRecorder::workerLoop() {
    while (true) {
        SaveTask task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            cvQueue.wait(lock, [this]() {
                return stopWorker || !taskQueue.empty();
            });

            if (stopWorker && taskQueue.empty()) {
                break;
            }

            task = std::move(taskQueue.front());
            taskQueue.pop();
        }

        if (task.type == TaskType::STREAM) {
            processStreamTask(task);
        } else {
            processSaveTask(task);
        }
    }
}

void FlightRecorder::enforceLimit() {
    try {
        std::vector<fs::path> replays;
        for (const auto& entry : fs::directory_iterator(outputDirectory)) {
            if (entry.is_directory()) {
                std::string name = entry.path().filename().string();
                if (name.rfind("shot_", 0) == 0 || name.rfind("stream_", 0) == 0) {
                    replays.push_back(entry.path());
                }
            }
        }

        std::sort(replays.begin(), replays.end());

        while (replays.size() > 10) {
            fs::path oldest = replays.front();
            spdlog::info("[FlightRecorder] Enforcing limit (count={}). Deleting oldest replay: {}", replays.size(), oldest.string());
            fs::remove_all(oldest);
            replays.erase(replays.begin());
        }
    } catch (const std::exception& e) {
        spdlog::error("[FlightRecorder] Error enforcing storage limits: {}", e.what());
    }
}

void FlightRecorder::saveSession(
    const std::vector<RecordedFrame>& frames,
    const LaunchData<Degrees, MilesPerHour>& launchData
) {
    if (frames.empty()) {
        spdlog::warn("[FlightRecorder] Attempted to save empty session, ignoring.");
        return;
    }

    // Prepare and clone task data in the consumer thread to decouple from state machine pool reuse
    SaveTask task;
    task.type = TaskType::SHOT;
    task.launchData = launchData;
    task.frames.reserve(frames.size());

    for (const auto& f : frames) {
        RecordedFrame clonedFrame;
        clonedFrame.timestamp = f.timestamp;
        clonedFrame.leftFrame = f.leftFrame.clone();   // Deep copy
        clonedFrame.rightFrame = f.rightFrame.clone(); // Deep copy
        clonedFrame.triggerDiag = f.triggerDiag;
        clonedFrame.leftVisionDiag = f.leftVisionDiag;
        clonedFrame.rightVisionDiag = f.rightVisionDiag;
        clonedFrame.triangulatedBalls = f.triangulatedBalls;
        task.frames.push_back(std::move(clonedFrame));
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        taskQueue.push(std::move(task));
    }
    cvQueue.notify_one();
}

void FlightRecorder::saveStreamSession(
    const std::vector<RecordedFrame>& frames
) {
    if (frames.empty()) {
        spdlog::warn("[FlightRecorder] Attempted to save empty stream session, ignoring.");
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt;
#ifdef _MSC_VER
    localtime_s(&bt, &timer);
#else
    localtime_r(&timer, &bt);
#endif

    std::ostringstream ssFolder;
    ssFolder << "stream_"
             << std::setfill('0')
             << std::setw(4) << (bt.tm_year + 1900)
             << std::setw(2) << (bt.tm_mon + 1)
             << std::setw(2) << bt.tm_mday << "_"
             << std::setw(2) << bt.tm_hour
             << std::setw(2) << bt.tm_min
             << std::setw(2) << bt.tm_sec << "_"
             << std::setw(3) << ms.count();

    SaveTask task;
    task.type = TaskType::STREAM;
    task.sessionTimestamp = ssFolder.str();
    task.frames.reserve(frames.size());

    for (const auto& f : frames) {
        RecordedFrame clonedFrame;
        clonedFrame.timestamp = f.timestamp;
        clonedFrame.leftFrame = f.leftFrame.clone();   // Deep copy
        clonedFrame.rightFrame = f.rightFrame.clone(); // Deep copy
        clonedFrame.triggerDiag = f.triggerDiag;
        clonedFrame.leftVisionDiag = f.leftVisionDiag;
        clonedFrame.rightVisionDiag = f.rightVisionDiag;
        clonedFrame.triangulatedBalls = f.triangulatedBalls;
        task.frames.push_back(std::move(clonedFrame));
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        taskQueue.push(std::move(task));
    }
    cvQueue.notify_one();
}

void FlightRecorder::processStreamTask(const SaveTask& task) {
    std::filesystem::path replayPath = std::filesystem::path(outputDirectory) / task.sessionTimestamp;
    std::filesystem::path rawPath = replayPath / "raw";
    std::filesystem::path annotatedPath = replayPath / "annotated";

    try {
        std::filesystem::create_directories(rawPath);
        std::filesystem::create_directories(annotatedPath);
    } catch (const std::exception& e) {
        spdlog::error("[FlightRecorder] Failed to create directories for stream session: {}", e.what());
        return;
    }

    nlohmann::json jMeta;
    jMeta["sessionType"] = "stream";
    jMeta["timestamp"] = task.sessionTimestamp;
    jMeta["frameCount"] = task.frames.size();
    jMeta["session"] = sessionInfo_;

    nlohmann::json jFrames = nlohmann::json::array();

    for (size_t i = 0; i < task.frames.size(); ++i) {
        const auto& f = task.frames[i];

        std::ostringstream ssIdx;
        ssIdx << std::setfill('0') << std::setw(3) << i;
        std::string filenameLeft = "left_" + ssIdx.str() + ".png";
        std::string filenameRight = "right_" + ssIdx.str() + ".png";

        if (!f.leftFrame.empty()) {
            cv::imwrite((rawPath / filenameLeft).string(), f.leftFrame);
            cv::imwrite((annotatedPath / filenameLeft).string(),
                        annotateFrame(f.leftFrame, f.triggerDiag, f.leftVisionDiag, f.triangulatedBalls, true));
        }
        if (!f.rightFrame.empty()) {
            cv::imwrite((rawPath / filenameRight).string(), f.rightFrame);
            cv::imwrite((annotatedPath / filenameRight).string(),
                        annotateFrame(f.rightFrame, f.triggerDiag, f.rightVisionDiag, f.triangulatedBalls, false));
        }

        nlohmann::json jFrame;
        jFrame["index"] = i;
        jFrame["timestamp"] = f.timestamp;
        jFrame["rawLeft"] = filenameLeft;
        jFrame["rawRight"] = filenameRight;
        jFrame["triggerDiag"] = f.triggerDiag;
        jFrame["leftVision"] = f.leftVisionDiag;
        jFrame["rightVision"] = f.rightVisionDiag;

        jFrames.push_back(jFrame);
    }

    jMeta["frames"] = jFrames;

    std::ofstream out((replayPath / "metadata.json").string());
    if (out.is_open()) {
        out << jMeta.dump(4);
        out.close();
        spdlog::info("[FlightRecorder] Saved stream session directory: {}", replayPath.string());
    } else {
        spdlog::error("[FlightRecorder] Failed to write metadata.json to {}", replayPath.string());
    }

    enforceLimit();
}

// =============================================================================
// Overlay drawing — one routine for both cameras and both session types
// =============================================================================

cv::Mat FlightRecorder::annotateFrame(const cv::Mat& gray,
                                      const nlohmann::json& triggerDiag,
                                      const nlohmann::json& visionDiag,
                                      const std::vector<Ball3D>& balls3D,
                                      bool isLeft) {
    cv::Mat ann;
    if (gray.channels() == 1) {
        cv::cvtColor(gray, ann, cv::COLOR_GRAY2BGR);
    } else {
        ann = gray.clone();
    }

    const cv::Scalar green(0, 255, 0), red(0, 0, 255), blue(255, 0, 0),
                     orange(0, 165, 255), cyan(255, 255, 0), yellow(0, 255, 255);

    auto rectFrom = [](const nlohmann::json& j, cv::Rect& out) {
        if (!j.is_array() || j.size() != 4) return false;
        out = cv::Rect(j[0].get<int>(), j[1].get<int>(), j[2].get<int>(), j[3].get<int>());
        return true;
    };

    // --- Trigger state (left camera carries the trigger's view) ---
    if (isLeft && triggerDiag.is_object()) {
        std::string st = triggerDiag.value("state", "");
        if (!st.empty()) {
            std::string text = "Trigger: " + st;
            if (triggerDiag.contains("stabilityCounter")) {
                text += " (" + std::to_string(triggerDiag.value("stabilityCounter", 0)) + "/" +
                        std::to_string(triggerDiag.value("lockFrameCount", 0)) + ")";
            } else if (triggerDiag.contains("searchStabilityCounter")) {
                text += " (stable " + std::to_string(triggerDiag.value("searchStabilityCounter", 0)) + ")";
            }
            if (triggerDiag.contains("matchScore")) {
                char buf[32];
                snprintf(buf, sizeof(buf), " match %.2f", triggerDiag.value("matchScore", 0.0f));
                text += buf;
            }
            if (triggerDiag.contains("disparityPx")) {
                char buf[64];
                snprintf(buf, sizeof(buf), " | dx %.0f dy %.0f px",
                         triggerDiag.value("disparityPx", 0.0), triggerDiag.value("verticalOffsetPx", 0.0));
                text += buf;
            }
            cv::putText(ann, text, cv::Point(20, ann.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, orange, 1);
        }

        cv::Rect lbox;
        if (triggerDiag.contains("lockedBallBox") && rectFrom(triggerDiag["lockedBallBox"], lbox) && lbox.width > 0) {
            cv::rectangle(ann, lbox, cyan, 2); // Cyan locked ball
        }
        cv::Rect gate;
        if (triggerDiag.contains("gateROI") && rectFrom(triggerDiag["gateROI"], gate)) {
            cv::rectangle(ann, gate, orange, 2);
        }
    }

    // --- Vision: dot clusters (candidates) and individual dots ---
    // The trigger publishes the finder's view too, so a stream recording made
    // before any shot still shows what was detected.
    const nlohmann::json* vision = &visionDiag;
    if ((!vision->is_object() || !vision->contains("candidates")) && triggerDiag.is_object()) {
        const char* key = isLeft ? "leftDots" : "rightDots";
        if (triggerDiag.contains(key)) {
            vision = &triggerDiag[key];
        } else if (triggerDiag.contains("dots")) {
            vision = &triggerDiag["dots"];
        }
    }

    if (vision->is_object() && vision->contains("dots")) {
        for (const auto& dJ : (*vision)["dots"]) {
            auto cJ = dJ["centroid"];
            if (!cJ.is_array() || cJ.size() != 2) continue;
            cv::Point centre(static_cast<int>(std::lround(cJ[0].get<double>())),
                             static_cast<int>(std::lround(cJ[1].get<double>())));
            if (dJ.value("accepted", false)) {
                cv::circle(ann, centre, 4, blue, 1);
            } else {
                cv::Rect box;
                if (dJ.contains("boundingBox") && rectFrom(dJ["boundingBox"], box)) {
                    cv::rectangle(ann, box, red, 1);
                } else {
                    cv::drawMarker(ann, centre, red, cv::MARKER_TILTED_CROSS, 6, 1);
                }
            }
        }
    }

    if (vision->is_object() && vision->contains("candidates")) {
        int noiseCount = 0;
        for (const auto& candJ : (*vision)["candidates"]) {
            bool accepted = candJ.value("accepted", false);
            cv::Rect bb;
            auto cenJ = candJ["centroid"];
            if (!rectFrom(candJ["boundingBox"], bb) || !cenJ.is_array() || cenJ.size() != 2) continue;
            cv::Point2d cen(cenJ[0].get<double>(), cenJ[1].get<double>());

            if (accepted) {
                cv::rectangle(ann, bb, green, 2);
                cv::line(ann, cv::Point2d(cen.x - 5, cen.y), cv::Point2d(cen.x + 5, cen.y), green, 2);
                cv::line(ann, cv::Point2d(cen.x, cen.y - 5), cv::Point2d(cen.x, cen.y + 5), green, 2);

                std::string lbl;
                if (candJ.contains("dotCount")) {
                    char buf[64];
                    snprintf(buf, sizeof(buf), "Ball (%d dots, %.0f px)",
                             candJ.value("dotCount", 0), candJ.value("spreadPx", 0.0));
                    lbl = buf;
                } else {
                    lbl = "Ball (A:" + std::to_string((int)candJ.value("area", 0.0)) + ")";
                }
                cv::putText(ann, lbl, cv::Point(bb.x, bb.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, green, 1);

                if (candJ.contains("markers")) {
                    for (const auto& mJ : candJ["markers"]) {
                        if (mJ.is_array() && mJ.size() == 2) {
                            cv::circle(ann, cv::Point2d(mJ[0].get<double>(), mJ[1].get<double>()), 2, blue, -1);
                        }
                    }
                }
            } else if (noiseCount < 15) {
                noiseCount++;
                cv::rectangle(ann, bb, red, 1);
                std::string lbl = "Noise: " + candJ.value("reason", std::string());
                cv::putText(ann, lbl, cv::Point(bb.x, bb.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, red, 1);
            }
        }
    }

    // --- Triangulated 3D coordinate ---
    if (!balls3D.empty()) {
        const auto& ball3D = balls3D[0];
        std::ostringstream spaceOss;
        spaceOss << "3D: (" << std::fixed << std::setprecision(3) << ball3D.centroid.x() << ", "
                 << ball3D.centroid.y() << ", " << ball3D.centroid.z() << ")";
        cv::putText(ann, spaceOss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, yellow, 2);
    }
    return ann;
}

void FlightRecorder::processSaveTask(const SaveTask& task) {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt;
#ifdef _MSC_VER
    localtime_s(&bt, &timer);
#else
    localtime_r(&timer, &bt);
#endif
    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y%m%d_%H%M%S") << "_" << std::setw(3) << std::setfill('0') << ms.count();
    std::string shotId = oss.str();

    fs::path replayPath = fs::path(outputDirectory) / ("shot_" + shotId);
    fs::path rawPath = replayPath / "raw";
    fs::path annPath = replayPath / "annotated";

    try {
        fs::create_directories(rawPath);
        fs::create_directories(annPath);
    } catch (const std::exception& e) {
        spdlog::error("[FlightRecorder] Failed to create directories for replay: {}", e.what());
        return;
    }

    nlohmann::json jMeta;
    jMeta["shotId"] = shotId;
    jMeta["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    jMeta["session"] = sessionInfo_;

    // Kinematics details
    jMeta["kinematics"] = {
        {"ballSpeed_mph", task.launchData.ballSpeed.value()},
        {"verticalLaunchAngle_deg", task.launchData.verticalLaunchAngle.value()},
        {"horizontalLaunchAngle_deg", task.launchData.horizontalLaunchAngle.value()},
        {"spinRPM", task.launchData.spinRPM},
        {"spinAxis", {task.launchData.spinAxis.x(), task.launchData.spinAxis.y(), task.launchData.spinAxis.z()}}
    };

    nlohmann::json jFrames = nlohmann::json::array();

    for (size_t i = 0; i < task.frames.size(); ++i) {
        const auto& rFrame = task.frames[i];
        std::ostringstream nameOss;
        nameOss << std::setw(3) << std::setfill('0') << i << ".png";
        std::string filename = nameOss.str();

        // Write raw frames
        if (!rFrame.leftFrame.empty()) {
            cv::imwrite((rawPath / ("left_" + filename)).string(), rFrame.leftFrame);
        }
        if (!rFrame.rightFrame.empty()) {
            cv::imwrite((rawPath / ("right_" + filename)).string(), rFrame.rightFrame);
        }

        // Draw annotations
        if (!rFrame.leftFrame.empty()) {
            cv::imwrite((annPath / ("left_" + filename)).string(),
                        annotateFrame(rFrame.leftFrame, rFrame.triggerDiag, rFrame.leftVisionDiag,
                                      rFrame.triangulatedBalls, true));
        }
        if (!rFrame.rightFrame.empty()) {
            cv::imwrite((annPath / ("right_" + filename)).string(),
                        annotateFrame(rFrame.rightFrame, rFrame.triggerDiag, rFrame.rightVisionDiag,
                                      rFrame.triangulatedBalls, false));
        }

        nlohmann::json jFrame;
        jFrame["index"] = i;
        jFrame["timestamp"] = rFrame.timestamp;
        jFrame["trigger"] = rFrame.triggerDiag;
        jFrame["leftVision"] = rFrame.leftVisionDiag;
        jFrame["rightVision"] = rFrame.rightVisionDiag;

        nlohmann::json j3DBalls = nlohmann::json::array();
        for (const auto& ball3D : rFrame.triangulatedBalls) {
            nlohmann::json jBall;
            jBall["centroid"] = {ball3D.centroid.x(), ball3D.centroid.y(), ball3D.centroid.z()};
            nlohmann::json jMarkers3D = nlohmann::json::array();
            for (const auto& m3D : ball3D.markers) {
                jMarkers3D.push_back({
                    {"position", {m3D.position.x(), m3D.position.y(), m3D.position.z()}},
                    {"confidence", m3D.confidence},
                    {"isStereo", m3D.isStereo}
                });
            }
            jBall["markers"] = jMarkers3D;
            j3DBalls.push_back(jBall);
        }
        jFrame["triangulatedBalls"] = j3DBalls;

        jFrames.push_back(jFrame);
    }

    jMeta["frames"] = jFrames;

    std::ofstream out((replayPath / "metadata.json").string());
    if (out.is_open()) {
        out << jMeta.dump(4);
        out.close();
        spdlog::info("[FlightRecorder] Saved session directory asynchronously: {}", replayPath.string());
    } else {
        spdlog::error("[FlightRecorder] Failed to write metadata.json to {}", replayPath.string());
    }

    enforceLimit();
}
