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

        processSaveTask(task);
    }
}

void FlightRecorder::enforceLimit() {
    try {
        std::vector<fs::path> replays;
        for (const auto& entry : fs::directory_iterator(outputDirectory)) {
            if (entry.is_directory()) {
                std::string name = entry.path().filename().string();
                if (name.rfind("shot_", 0) == 0) {
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
        task.frames.push_back(clonedFrame);
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        taskQueue.push(task);
    }
    cvQueue.notify_one();
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
        cv::Mat annLeft, annRight;
        if (!rFrame.leftFrame.empty()) {
            cv::cvtColor(rFrame.leftFrame, annLeft, cv::COLOR_GRAY2BGR);
            
            // Draw trigger box if data is valid
            if (rFrame.triggerDiag.contains("gateROI")) {
                auto roiJ = rFrame.triggerDiag["gateROI"];
                if (roiJ.is_array() && roiJ.size() == 4) {
                    cv::Rect roi(roiJ[0], roiJ[1], roiJ[2], roiJ[3]);
                    cv::rectangle(annLeft, roi, cv::Scalar(0, 165, 255), 2);
                    std::string triggerText = "Gate Pixels: " + std::to_string(rFrame.triggerDiag.value("nonZeroCount", 0)) +
                                              " / " + std::to_string(rFrame.triggerDiag.value("minBallPixels", 0));
                    cv::putText(annLeft, triggerText, cv::Point(roi.x, roi.y - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 165, 255), 1);
                }
            }
            
            // Draw candidates
            if (rFrame.leftVisionDiag.contains("candidates")) {
                for (const auto& candJ : rFrame.leftVisionDiag["candidates"]) {
                    bool accepted = candJ.value("accepted", false);
                    auto bbJ = candJ["boundingBox"];
                    auto cenJ = candJ["centroid"];
                    if (bbJ.is_array() && bbJ.size() == 4 && cenJ.is_array() && cenJ.size() == 2) {
                        cv::Rect bb(bbJ[0], bbJ[1], bbJ[2], bbJ[3]);
                        cv::Point2d cen(cenJ[0], cenJ[1]);
                        
                        if (accepted) {
                            cv::rectangle(annLeft, bb, cv::Scalar(0, 255, 0), 2);
                            cv::line(annLeft, cv::Point2d(cen.x - 5, cen.y), cv::Point2d(cen.x + 5, cen.y), cv::Scalar(0, 255, 0), 2);
                            cv::line(annLeft, cv::Point2d(cen.x, cen.y - 5), cv::Point2d(cen.x, cen.y + 5), cv::Scalar(0, 255, 0), 2);
                            
                            std::string lbl = "Ball (A:" + std::to_string((int)candJ.value("area", 0.0)) + ")";
                            cv::putText(annLeft, lbl, cv::Point(bb.x, bb.y - 5),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                            // Draw markers
                            if (candJ.contains("markers")) {
                                for (const auto& mJ : candJ["markers"]) {
                                    if (mJ.is_array() && mJ.size() == 2) {
                                        cv::circle(annLeft, cv::Point2d(mJ[0], mJ[1]), 2, cv::Scalar(255, 0, 0), -1);
                                    }
                                }
                            }
                        } else {
                            cv::rectangle(annLeft, bb, cv::Scalar(0, 0, 255), 1);
                            std::string lbl = "Noise: " + candJ.value("reason", "");
                            cv::putText(annLeft, lbl, cv::Point(bb.x, bb.y - 5),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
                        }
                    }
                }
            }

            // Draw triangulated 3D coordinate on left overlay if present
            if (!rFrame.triangulatedBalls.empty()) {
                const auto& ball3D = rFrame.triangulatedBalls[0];
                std::ostringstream spaceOss;
                spaceOss << "3D: (" << std::fixed << std::setprecision(3) << ball3D.centroid.x() << ", "
                         << ball3D.centroid.y() << ", " << ball3D.centroid.z() << ")";
                cv::putText(annLeft, spaceOss.str(), cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
            cv::imwrite((annPath / ("left_" + filename)).string(), annLeft);
        }

        if (!rFrame.rightFrame.empty()) {
            cv::cvtColor(rFrame.rightFrame, annRight, cv::COLOR_GRAY2BGR);
            
            // Draw candidates
            if (rFrame.rightVisionDiag.contains("candidates")) {
                for (const auto& candJ : rFrame.rightVisionDiag["candidates"]) {
                    bool accepted = candJ.value("accepted", false);
                    auto bbJ = candJ["boundingBox"];
                    auto cenJ = candJ["centroid"];
                    if (bbJ.is_array() && bbJ.size() == 4 && cenJ.is_array() && cenJ.size() == 2) {
                        cv::Rect bb(bbJ[0], bbJ[1], bbJ[2], bbJ[3]);
                        cv::Point2d cen(cenJ[0], cenJ[1]);
                        
                        if (accepted) {
                            cv::rectangle(annRight, bb, cv::Scalar(0, 255, 0), 2);
                            cv::line(annRight, cv::Point2d(cen.x - 5, cen.y), cv::Point2d(cen.x + 5, cen.y), cv::Scalar(0, 255, 0), 2);
                            cv::line(annRight, cv::Point2d(cen.x, cen.y - 5), cv::Point2d(cen.x, cen.y + 5), cv::Scalar(0, 255, 0), 2);
                            
                            std::string lbl = "Ball (A:" + std::to_string((int)candJ.value("area", 0.0)) + ")";
                            cv::putText(annRight, lbl, cv::Point(bb.x, bb.y - 5),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                            // Draw markers
                            if (candJ.contains("markers")) {
                                for (const auto& mJ : candJ["markers"]) {
                                    if (mJ.is_array() && mJ.size() == 2) {
                                        cv::circle(annRight, cv::Point2d(mJ[0], mJ[1]), 2, cv::Scalar(255, 0, 0), -1);
                                    }
                                }
                            }
                        } else {
                            cv::rectangle(annRight, bb, cv::Scalar(0, 0, 255), 1);
                            std::string lbl = "Noise: " + candJ.value("reason", "");
                            cv::putText(annRight, lbl, cv::Point(bb.x, bb.y - 5),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
                        }
                    }
                }
            }

            // Draw triangulated 3D coordinate on right overlay if present
            if (!rFrame.triangulatedBalls.empty()) {
                const auto& ball3D = rFrame.triangulatedBalls[0];
                std::ostringstream spaceOss;
                spaceOss << "3D: (" << std::fixed << std::setprecision(3) << ball3D.centroid.x() << ", "
                         << ball3D.centroid.y() << ", " << ball3D.centroid.z() << ")";
                cv::putText(annRight, spaceOss.str(), cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
            cv::imwrite((annPath / ("right_" + filename)).string(), annRight);
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
