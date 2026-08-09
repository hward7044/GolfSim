#include "Math/OpenCVMomentsTracker.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

OpenCVMomentsTracker::OpenCVMomentsTracker(int ballThresh, int markerThresh, double minBAr, double maxBAr, double minBCirc)
    : ballThreshold_(ballThresh),
      markerThreshold_(markerThresh),
      minBallArea_(minBAr),
      maxBallArea_(maxBAr),
      minBallCircularity_(minBCirc) {
    latestDiag = {{"candidates", nlohmann::json::array()}};
}

// Helper to extract markers within a specific ball region
std::vector<MarkerObservation> OpenCVMomentsTracker::extractMarkersInROI(
    const cv::Mat& gray, 
    const cv::Rect& roi, 
    int markerThreshold
) {
    std::vector<MarkerObservation> markers;
    
    // Ensure ROI is valid
    cv::Rect safeRoi = roi & cv::Rect(0, 0, gray.cols, gray.rows);
    if (safeRoi.area() <= 0) {
        return markers;
    }

    cv::Mat localRoi = gray(safeRoi);
    thresh_.create(localRoi.size(), CV_8UC1);
    cv::threshold(localRoi, thresh_, markerThreshold, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> markerContours;
    cv::findContours(thresh_, markerContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& mc : markerContours) {
        double area = cv::contourArea(mc);
        // Markers should be small glints (e.g., between 1 and 80 pixels)
        if (area < 1.0 || area > 80.0) {
            continue;
        }

        cv::Moments m = cv::moments(mc);
        if (m.m00 > 0.0) {
            cv::Point2d localCentroid(m.m10 / m.m00, m.m01 / m.m00);
            cv::Point2d globalCentroid(safeRoi.x + localCentroid.x, safeRoi.y + localCentroid.y);
            
            // Get peak intensity
            cv::Rect mcRect = cv::boundingRect(mc);
            double maxVal = 0.0;
            cv::minMaxLoc(localRoi(mcRect), nullptr, &maxVal, nullptr, nullptr);

            MarkerObservation obs;
            obs.position = globalCentroid;
            obs.intensity = maxVal;
            markers.push_back(obs);
        }
    }
    return markers;
}

std::vector<BallObservation> OpenCVMomentsTracker::detectBalls(const cv::Mat& frame) {
    std::vector<BallObservation> ballObservations;
    nlohmann::json candidates = nlohmann::json::array();

    if (frame.empty()) {
        latestDiag = {{"candidates", candidates}};
        return ballObservations;
    }

    // Convert to grayscale
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray_, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray_, cv::COLOR_BGRA2GRAY);
    } else {
        gray_ = frame;
    }

    // Threshold for ball silhouette
    cv::threshold(gray_, ballMask_, ballThreshold_, 255, cv::THRESH_BINARY);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(ballMask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        cv::Rect boundRect = cv::boundingRect(contour);
        cv::Point2d centroid(boundRect.x + boundRect.width / 2.0, boundRect.y + boundRect.height / 2.0);

        if (area < minBallArea_ || area > maxBallArea_) {
            nlohmann::json cand;
            cand["centroid"] = {centroid.x, centroid.y};
            cand["boundingBox"] = {boundRect.x, boundRect.y, boundRect.width, boundRect.height};
            cand["area"] = area;
            cand["circularity"] = 0.0;
            cand["isOverlapping"] = false;
            cand["accepted"] = false;
            cand["reason"] = area < minBallArea_ ? "Area too small" : "Area too large";
            cand["markers"] = nlohmann::json::array();
            candidates.push_back(cand);
            continue;
        }

        double perimeter = cv::arcLength(contour, true);
        double circularity = 0.0;
        if (perimeter > 0.0) {
            circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
        }

        // Detect if this is likely overlapping ball silhouettes (wedge shot extreme case)
        bool isOverlapping = (area > 1.5 * minBallArea_ && circularity < minBallCircularity_);

        if (isOverlapping) {
            // Pad slightly
            cv::Rect searchRect = boundRect;
            searchRect.x = std::max(0, searchRect.x - 10);
            searchRect.y = std::max(0, searchRect.y - 10);
            searchRect.width = std::min(gray_.cols - searchRect.x, searchRect.width + 20);
            searchRect.height = std::min(gray_.rows - searchRect.y, searchRect.height + 20);

            localRegion_ = gray_(searchRect);
            
            // Reduce high-frequency noise before Hough Circle transform
            cv::GaussianBlur(localRegion_, localRegionBlurred_, cv::Size(3, 3), 0.5);
            
            // Hough Circle Transform to isolate overlapping circles
            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(localRegionBlurred_, circles, cv::HOUGH_GRADIENT, 1.0, 15.0, 50.0, 35.0, 12, 55);

            if (circles.empty()) {
                nlohmann::json cand;
                cand["centroid"] = {centroid.x, centroid.y};
                cand["boundingBox"] = {boundRect.x, boundRect.y, boundRect.width, boundRect.height};
                cand["area"] = area;
                cand["circularity"] = circularity;
                cand["isOverlapping"] = true;
                cand["accepted"] = false;
                cand["reason"] = "Overlapping contour: no Hough circles found";
                cand["markers"] = nlohmann::json::array();
                candidates.push_back(cand);
            } else {
                for (const auto& c : circles) {
                    cv::Point2d globalCentroid(searchRect.x + c[0], searchRect.y + c[1]);
                    double radius = c[2];
                    cv::Rect ballRoi(
                        static_cast<int>(globalCentroid.x - radius),
                        static_cast<int>(globalCentroid.y - radius),
                        static_cast<int>(2 * radius),
                        static_cast<int>(2 * radius)
                    );
                    cv::Rect clippedRoi = ballRoi & cv::Rect(0, 0, gray_.cols, gray_.rows);

                    BallObservation obs;
                    obs.centroid = globalCentroid;
                    obs.boundingBox = clippedRoi;
                    obs.markers = extractMarkersInROI(gray_, obs.boundingBox, markerThreshold_);
                    ballObservations.push_back(obs);

                    nlohmann::json cand;
                    cand["centroid"] = {globalCentroid.x, globalCentroid.y};
                    cand["boundingBox"] = {clippedRoi.x, clippedRoi.y, clippedRoi.width, clippedRoi.height};
                    cand["area"] = area;
                    cand["circularity"] = circularity;
                    cand["isOverlapping"] = true;
                    cand["accepted"] = true;
                    cand["reason"] = "Accepted (Hough Circle)";
                    
                    nlohmann::json jMarkers = nlohmann::json::array();
                    for (const auto& m : obs.markers) {
                        jMarkers.push_back({m.position.x, m.position.y});
                    }
                    cand["markers"] = jMarkers;
                    candidates.push_back(cand);
                }
            }
        } else {
            if (circularity < minBallCircularity_) {
                nlohmann::json cand;
                cand["centroid"] = {centroid.x, centroid.y};
                cand["boundingBox"] = {boundRect.x, boundRect.y, boundRect.width, boundRect.height};
                cand["area"] = area;
                cand["circularity"] = circularity;
                cand["isOverlapping"] = false;
                cand["accepted"] = false;
                cand["reason"] = "Circularity too low";
                cand["markers"] = nlohmann::json::array();
                candidates.push_back(cand);
                continue;
            }

            // Process contour passing circularity threshold using moments centroid tracking
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0.0) {
                cv::Point2d momentsCentroid(m.m10 / m.m00, m.m01 / m.m00);
                cv::Rect boundRect = cv::boundingRect(contour);

                BallObservation obs;
                obs.centroid = momentsCentroid;
                obs.boundingBox = boundRect;
                obs.markers = extractMarkersInROI(gray_, boundRect, markerThreshold_);
                ballObservations.push_back(obs);

                nlohmann::json cand;
                cand["centroid"] = {momentsCentroid.x, momentsCentroid.y};
                cand["boundingBox"] = {boundRect.x, boundRect.y, boundRect.width, boundRect.height};
                cand["area"] = area;
                cand["circularity"] = circularity;
                cand["isOverlapping"] = false;
                cand["accepted"] = true;
                cand["reason"] = "Accepted (Moments)";
                
                nlohmann::json jMarkers = nlohmann::json::array();
                for (const auto& m : obs.markers) {
                    jMarkers.push_back({m.position.x, m.position.y});
                }
                cand["markers"] = jMarkers;
                candidates.push_back(cand);
            } else {
                nlohmann::json cand;
                cand["centroid"] = {centroid.x, centroid.y};
                cand["boundingBox"] = {boundRect.x, boundRect.y, boundRect.width, boundRect.height};
                cand["area"] = area;
                cand["circularity"] = circularity;
                cand["isOverlapping"] = false;
                cand["accepted"] = false;
                cand["reason"] = "Moments calculation failed (m00 == 0)";
                cand["markers"] = nlohmann::json::array();
                candidates.push_back(cand);
            }
        }
    }

    latestDiag = {{"candidates", candidates}};
    return ballObservations;
}
