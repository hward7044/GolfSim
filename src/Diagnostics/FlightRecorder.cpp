#include "Diagnostics/FlightRecorder.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

FlightRecorder::FlightRecorder(const std::string& outDir)
    : outputDirectory(outDir) {
    try {
        fs::create_directories(outputDirectory);
    } catch (const std::exception& e) {
        spdlog::error("[FlightRecorder] Failed to create output directory {}: {}", outputDirectory, e.what());
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

        // Lexicographical sort on shot_YYYYMMDD_HHMMSS_mmm naturally sorts chronologically
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

    // 1. Generate unique shot ID based on time
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
        {"ballSpeed_mph", launchData.ballSpeed.value()},
        {"verticalLaunchAngle_deg", launchData.verticalLaunchAngle.value()},
        {"horizontalLaunchAngle_deg", launchData.horizontalLaunchAngle.value()},
        {"spinRPM", launchData.spinRPM},
        {"spinAxis", {launchData.spinAxis.x(), launchData.spinAxis.y(), launchData.spinAxis.z()}}
    };

    nlohmann::json jFrames = nlohmann::json::array();

    // 2. Loop through frames and write
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& rFrame = frames[i];
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
            
            // Draw trigger box
            cv::rectangle(annLeft, rFrame.triggerDiag.gateROI, cv::Scalar(0, 165, 255), 2);
            std::string triggerText = "Gate Pixels: " + std::to_string(rFrame.triggerDiag.nonZeroCount) +
                                      " / " + std::to_string(rFrame.triggerDiag.minBallPixels);
            cv::putText(annLeft, triggerText, cv::Point(rFrame.triggerDiag.gateROI.x, rFrame.triggerDiag.gateROI.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 165, 255), 1);
            
            // Draw candidates
            for (const auto& cand : rFrame.leftVisionDiag.candidates) {
                if (cand.accepted) {
                    cv::rectangle(annLeft, cand.boundingBox, cv::Scalar(0, 255, 0), 2);
                    cv::line(annLeft, cv::Point2d(cand.centroid.x - 5, cand.centroid.y), cv::Point2d(cand.centroid.x + 5, cand.centroid.y), cv::Scalar(0, 255, 0), 2);
                    cv::line(annLeft, cv::Point2d(cand.centroid.x, cand.centroid.y - 5), cv::Point2d(cand.centroid.x, cand.centroid.y + 5), cv::Scalar(0, 255, 0), 2);
                    
                    std::string lbl = "Ball (A:" + std::to_string((int)cand.area) + ", C:" + std::to_string(cand.circularity).substr(0, 4) + ")";
                    cv::putText(annLeft, lbl, cv::Point(cand.boundingBox.x, cand.boundingBox.y - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                    // Draw markers inside candidate
                    for (const auto& mPos : cand.markers) {
                        cv::circle(annLeft, mPos, 2, cv::Scalar(255, 0, 0), -1); // blue marker dot
                    }
                } else {
                    cv::rectangle(annLeft, cand.boundingBox, cv::Scalar(0, 0, 255), 1);
                    std::string lbl = "Noise: " + cand.reason;
                    cv::putText(annLeft, lbl, cv::Point(cand.boundingBox.x, cand.boundingBox.y - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
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
            for (const auto& cand : rFrame.rightVisionDiag.candidates) {
                if (cand.accepted) {
                    cv::rectangle(annRight, cand.boundingBox, cv::Scalar(0, 255, 0), 2);
                    cv::line(annRight, cv::Point2d(cand.centroid.x - 5, cand.centroid.y), cv::Point2d(cand.centroid.x + 5, cand.centroid.y), cv::Scalar(0, 255, 0), 2);
                    cv::line(annRight, cv::Point2d(cand.centroid.x, cand.centroid.y - 5), cv::Point2d(cand.centroid.x, cand.centroid.y + 5), cv::Scalar(0, 255, 0), 2);
                    
                    std::string lbl = "Ball (A:" + std::to_string((int)cand.area) + ", C:" + std::to_string(cand.circularity).substr(0, 4) + ")";
                    cv::putText(annRight, lbl, cv::Point(cand.boundingBox.x, cand.boundingBox.y - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                    // Draw markers inside candidate
                    for (const auto& mPos : cand.markers) {
                        cv::circle(annRight, mPos, 2, cv::Scalar(255, 0, 0), -1); // blue marker dot
                    }
                } else {
                    cv::rectangle(annRight, cand.boundingBox, cv::Scalar(0, 0, 255), 1);
                    std::string lbl = "Noise: " + cand.reason;
                    cv::putText(annRight, lbl, cv::Point(cand.boundingBox.x, cand.boundingBox.y - 5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
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

        // Construct JSON frame representation
        nlohmann::json jFrame;
        jFrame["index"] = i;
        jFrame["timestamp"] = rFrame.timestamp;
        
        jFrame["trigger"] = {
            {"nonZeroCount", rFrame.triggerDiag.nonZeroCount},
            {"minBallPixels", rFrame.triggerDiag.minBallPixels},
            {"pixelDiffThreshold", rFrame.triggerDiag.pixelDiffThreshold},
            {"triggered", rFrame.triggerDiag.triggered},
            {"gateROI", {rFrame.triggerDiag.gateROI.x, rFrame.triggerDiag.gateROI.y, rFrame.triggerDiag.gateROI.width, rFrame.triggerDiag.gateROI.height}}
        };

        auto serializeCandidates = [](const VisionDiagnostics& vd) {
            nlohmann::json cands = nlohmann::json::array();
            for (const auto& cand : vd.candidates) {
                nlohmann::json jCand;
                jCand["centroid"] = {cand.centroid.x, cand.centroid.y};
                jCand["boundingBox"] = {cand.boundingBox.x, cand.boundingBox.y, cand.boundingBox.width, cand.boundingBox.height};
                jCand["area"] = cand.area;
                jCand["circularity"] = cand.circularity;
                jCand["isOverlapping"] = cand.isOverlapping;
                jCand["accepted"] = cand.accepted;
                jCand["reason"] = cand.reason;
                
                nlohmann::json jMarkers = nlohmann::json::array();
                for (const auto& m : cand.markers) {
                    jMarkers.push_back({m.x, m.y});
                }
                jCand["markers"] = jMarkers;
                cands.push_back(jCand);
            }
            return cands;
        };

        jFrame["leftVision"] = serializeCandidates(rFrame.leftVisionDiag);
        jFrame["rightVision"] = serializeCandidates(rFrame.rightVisionDiag);

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

    // Write metadata to file
    std::ofstream out((replayPath / "metadata.json").string());
    if (out.is_open()) {
        out << jMeta.dump(4);
        out.close();
        spdlog::info("[FlightRecorder] Saved session directory: {}", replayPath.string());
    } else {
        spdlog::error("[FlightRecorder] Failed to write metadata.json to {}", replayPath.string());
    }

    // Enforce 10 replays limit
    enforceLimit();
}
