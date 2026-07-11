#include "Math/OpenCVMomentsTracker.hpp"
#include <cmath>

OpenCVMomentsTracker::OpenCVMomentsTracker(int ballThresh, int markerThresh, double minBAr, double maxBAr, double minBCirc)
    : ballThreshold_(ballThresh),
      markerThreshold_(markerThresh),
      minBallArea_(minBAr),
      maxBallArea_(maxBAr),
      minBallCircularity_(minBCirc) {}

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
    if (frame.empty()) {
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
        if (area < minBallArea_ || area > maxBallArea_) {
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
            cv::Rect boundRect = cv::boundingRect(contour);
            // Pad slightly
            boundRect.x = std::max(0, boundRect.x - 10);
            boundRect.y = std::max(0, boundRect.y - 10);
            boundRect.width = std::min(gray_.cols - boundRect.x, boundRect.width + 20);
            boundRect.height = std::min(gray_.rows - boundRect.y, boundRect.height + 20);

            localRegion_ = gray_(boundRect);
            
            // Reduce high-frequency noise before Hough Circle transform
            cv::GaussianBlur(localRegion_, localRegionBlurred_, cv::Size(3, 3), 0.5);
            
            // Hough Circle Transform to isolate overlapping circles
            std::vector<cv::Vec3f> circles;
            // Expected ball radius in pixels (approx 15 to 50 pixels)
            // Use param2 = 35.0 (up from 20.0) to reduce false positives
            cv::HoughCircles(localRegionBlurred_, circles, cv::HOUGH_GRADIENT, 1.0, 15.0, 50.0, 35.0, 12, 55);

            for (const auto& c : circles) {
                cv::Point2d globalCentroid(boundRect.x + c[0], boundRect.y + c[1]);
                double radius = c[2];
                cv::Rect ballRoi(
                    static_cast<int>(globalCentroid.x - radius),
                    static_cast<int>(globalCentroid.y - radius),
                    static_cast<int>(2 * radius),
                    static_cast<int>(2 * radius)
                );

                BallObservation obs;
                obs.centroid = globalCentroid;
                obs.boundingBox = ballRoi & cv::Rect(0, 0, gray_.cols, gray_.rows);
                obs.markers = extractMarkersInROI(gray_, obs.boundingBox, markerThreshold_);
                ballObservations.push_back(obs);
            }
        } else {
            // Process contour. Even if circularity is slightly below threshold, we still fallback to 
            // moments centroid tracking instead of silently discarding it.
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0.0) {
                cv::Point2d centroid(m.m10 / m.m00, m.m01 / m.m00);
                cv::Rect boundRect = cv::boundingRect(contour);

                BallObservation obs;
                obs.centroid = centroid;
                obs.boundingBox = boundRect;
                obs.markers = extractMarkersInROI(gray_, boundRect, markerThreshold_);
                ballObservations.push_back(obs);
            }
        }
    }

    return ballObservations;
}
