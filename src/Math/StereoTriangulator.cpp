#include "Math/StereoTriangulator.hpp"
#include <algorithm>
#include <cmath>

StereoTriangulator::StereoTriangulator() : ballRadius_(0.021335) {
    // Default calibration parameters mapping to a horizontal stereo setup with 100mm baseline
    calib_.K_L = (cv::Mat_<double>(3, 3) << 1000.0, 0.0, 640.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 1.0);
    calib_.D_L = cv::Mat::zeros(1, 5, CV_64F);
    
    calib_.K_R = calib_.K_L.clone();
    calib_.D_R = calib_.D_L.clone();

    calib_.R = cv::Mat::eye(3, 3, CV_64F);
    calib_.T = (cv::Mat_<double>(3, 1) << -0.1, 0.0, 0.0); // 100mm baseline along X

    calib_.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib_.R_R = cv::Mat::eye(3, 3, CV_64F);

    // Identity projection matrices
    calib_.P_L = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    calib_.P_R = (cv::Mat_<double>(3, 4) << 1000.0, 0.0, 640.0, -100.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

StereoTriangulator::StereoTriangulator(const StereoCalibration& calib, double ballRadius)
    : calib_(calib), ballRadius_(ballRadius) {}

void StereoTriangulator::setCalibration(const StereoCalibration& calib) {
    calib_ = calib;
}

static Eigen::Vector3d toEigenVec(const cv::Mat& m) {
    return Eigen::Vector3d(m.at<double>(0, 0), m.at<double>(1, 0), m.at<double>(2, 0));
}

std::vector<Ball3D> StereoTriangulator::triangulateShot(
    const std::vector<BallObservation>& leftObs,
    const std::vector<BallObservation>& rightObs
) {
    std::vector<Ball3D> trajectory;

    // 1. Rectify and project all ball centroids to rectified space
    std::vector<cv::Point2d> rectLeft(leftObs.size());
    pt_temp_.create(1, 1, CV_64FC2);
    for (std::size_t i = 0; i < leftObs.size(); ++i) {
        pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(leftObs[i].centroid.x, leftObs[i].centroid.y);
        cv::undistortPoints(pt_temp_, und_temp_, calib_.K_L, calib_.D_L, calib_.R_L, calib_.P_L);
        rectLeft[i] = cv::Point2d(und_temp_.at<double>(0, 0), und_temp_.at<double>(0, 1));
    }

    std::vector<cv::Point2d> rectRight(rightObs.size());
    for (std::size_t j = 0; j < rightObs.size(); ++j) {
        pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(rightObs[j].centroid.x, rightObs[j].centroid.y);
        cv::undistortPoints(pt_temp_, und_temp_, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);
        rectRight[j] = cv::Point2d(und_temp_.at<double>(0, 0), und_temp_.at<double>(0, 1));
    }

    // 2. Sort indices horizontally (downrange along X) to preserve chronological path structure
    std::vector<std::size_t> sortedLeftIdx(leftObs.size());
    for (std::size_t i = 0; i < leftObs.size(); ++i) sortedLeftIdx[i] = i;
    std::sort(sortedLeftIdx.begin(), sortedLeftIdx.end(), [&](std::size_t a, std::size_t b) {
        return rectLeft[a].x < rectLeft[b].x;
    });

    std::vector<std::size_t> sortedRightIdx(rightObs.size());
    for (std::size_t j = 0; j < rightObs.size(); ++j) sortedRightIdx[j] = j;
    std::sort(sortedRightIdx.begin(), sortedRightIdx.end(), [&](std::size_t a, std::size_t b) {
        return rectRight[a].x < rectRight[b].x;
    });

    // 3. Match Left and Right centroids using epipolar and disparity constraints
    std::vector<std::pair<std::size_t, std::size_t>> matches;
    std::vector<bool> rightMatched(rightObs.size(), false);

    for (std::size_t i : sortedLeftIdx) {
        cv::Point2d pl = rectLeft[i];
        int bestJ = -1;
        double bestDiffY = 4.0; // 4 pixels vertical alignment tolerance in rectified space

        for (std::size_t j : sortedRightIdx) {
            if (rightMatched[j]) continue;
            cv::Point2d pr = rectRight[j];

            double diffY = std::abs(pl.y - pr.y);
            double disparity = std::abs(pl.x - pr.x);

            // Disparity must fall in a realistic physical range (e.g., 10 to 300 pixels)
            if (diffY < bestDiffY && disparity > 10.0 && disparity < 300.0) {
                bestDiffY = diffY;
                bestJ = static_cast<int>(j);
            }
        }

        if (bestJ != -1) {
            rightMatched[bestJ] = true;
            matches.push_back({i, static_cast<std::size_t>(bestJ)});
        }
    }

    // Sort matches chronologically based on Left observation X position
    std::sort(matches.begin(), matches.end(), [&](const auto& a, const auto& b) {
        return rectLeft[a.first].x < rectLeft[b.first].x;
    });

    // 4. Triangulate the matched pairs
    for (const auto& match : matches) {
        const auto& bL = leftObs[match.first];
        const auto& bR = rightObs[match.second];

        // Triangulate ball centroid
        pt2D_L_.create(1, 1, CV_64FC2);
        pt2D_L_.at<cv::Vec2d>(0, 0) = cv::Vec2d(bL.centroid.x, bL.centroid.y);
        pt2D_R_.create(1, 1, CV_64FC2);
        pt2D_R_.at<cv::Vec2d>(0, 0) = cv::Vec2d(bR.centroid.x, bR.centroid.y);

        cv::undistortPoints(pt2D_L_, undL_, calib_.K_L, calib_.D_L, calib_.R_L, calib_.P_L);
        cv::undistortPoints(pt2D_R_, undR_, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);

        cv::triangulatePoints(calib_.P_L, calib_.P_R, undL_, undR_, pt4D_);
        double w = pt4D_.at<double>(3, 0);
        if (std::abs(w) < 1e-6) continue;

        Eigen::Vector3d ballCentroid3D(
            pt4D_.at<double>(0, 0) / w,
            pt4D_.at<double>(1, 0) / w,
            pt4D_.at<double>(2, 0) / w
        );

        Ball3D ball3D;
        ball3D.centroid = ballCentroid3D;

        // Triangulate/Reconstruct Markers
        std::vector<bool> rightMarkerUsed(bR.markers.size(), false);
        std::vector<Marker3D> markers3D;

        // Prep Right camera marker undistorted rectified positions for epipolar matching
        std::vector<cv::Point2d> undMrList;
        undMrList.reserve(bR.markers.size());
        for (const auto& mr : bR.markers) {
            pt_temp_.create(1, 1, CV_64FC2);
            pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(mr.position.x, mr.position.y);
            cv::undistortPoints(pt_temp_, und_temp_, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);
            undMrList.push_back(cv::Point2d(und_temp_.at<double>(0, 0), und_temp_.at<double>(0, 1)));
        }

        // Try to match Left markers with Right markers
        for (const auto& ml : bL.markers) {
            pt_temp_.create(1, 1, CV_64FC2);
            pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(ml.position.x, ml.position.y);
            cv::undistortPoints(pt_temp_, und_temp_, calib_.K_L, calib_.D_L, calib_.R_L, calib_.P_L);
            cv::Point2d undMl(und_temp_.at<double>(0, 0), und_temp_.at<double>(0, 1));

            int bestMatchIndex = -1;
            double bestMatchDiffY = 3.0; // 3.0 pixels rectified y tolerance

            for (std::size_t r = 0; r < bR.markers.size(); ++r) {
                if (rightMarkerUsed[r]) continue;
                double diffY = std::abs(undMl.y - undMrList[r].y);
                if (diffY < bestMatchDiffY) {
                    bestMatchDiffY = diffY;
                    bestMatchIndex = static_cast<int>(r);
                }
            }

            if (bestMatchIndex != -1) {
                // Found stereo match! Triangulate standard 3D point.
                rightMarkerUsed[bestMatchIndex] = true;
                
                pt2D_L_.create(1, 1, CV_64FC2);
                pt2D_L_.at<cv::Vec2d>(0, 0) = cv::Vec2d(undMl.x, undMl.y);
                pt2D_R_.create(1, 1, CV_64FC2);
                pt2D_R_.at<cv::Vec2d>(0, 0) = cv::Vec2d(undMrList[bestMatchIndex].x, undMrList[bestMatchIndex].y);

                cv::triangulatePoints(calib_.P_L, calib_.P_R, pt2D_L_, pt2D_R_, pt4D_);
                double mw = pt4D_.at<double>(3, 0);
                if (std::abs(mw) > 1e-6) {
                    Marker3D marker3D;
                    marker3D.position = Eigen::Vector3d(
                        pt4D_.at<double>(0, 0) / mw,
                        pt4D_.at<double>(1, 0) / mw,
                        pt4D_.at<double>(2, 0) / mw
                    );
                    marker3D.confidence = 1.0; // High confidence
                    marker3D.isStereo = true;
                    markers3D.push_back(marker3D);
                }
            } else {
                // Ray-sphere intersection for unmatched left marker
                pt_temp_.create(1, 1, CV_64FC2);
                pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(ml.position.x, ml.position.y);
                cv::undistortPoints(pt_temp_, und_temp_, calib_.K_L, calib_.D_L);
                
                double x_norm = und_temp_.at<double>(0, 0);
                double y_norm = und_temp_.at<double>(0, 1);
                
                // Ray in Left camera frame (world frame)
                Eigen::Vector3d rayDir(x_norm, y_norm, 1.0);
                rayDir.normalize();

                Eigen::Vector3d O(0.0, 0.0, 0.0); // Origin of left camera
                Eigen::Vector3d w_vec = O - ballCentroid3D;

                double dot = w_vec.dot(rayDir);
                double disc = dot * dot - (w_vec.squaredNorm() - ballRadius_ * ballRadius_);
                if (disc >= 0.0) {
                    double t = -dot - std::sqrt(disc);
                    Marker3D marker3D;
                    marker3D.position = O + t * rayDir;
                    marker3D.confidence = 0.5; // Low confidence
                    marker3D.isStereo = false;
                    markers3D.push_back(marker3D);
                }
            }
        }

        // Ray-sphere intersection for unmatched right markers
        for (std::size_t r = 0; r < bR.markers.size(); ++r) {
            if (rightMarkerUsed[r]) continue;

            const auto& mr = bR.markers[r];
            pt_temp_.create(1, 1, CV_64FC2);
            pt_temp_.at<cv::Vec2d>(0, 0) = cv::Vec2d(mr.position.x, mr.position.y);
            cv::undistortPoints(pt_temp_, und_temp_, calib_.K_R, calib_.D_R);

            double x_norm = und_temp_.at<double>(0, 0);
            double y_norm = und_temp_.at<double>(0, 1);

            // Ray in Right camera frame
            Eigen::Vector3d rayCamR(x_norm, y_norm, 1.0);
            rayCamR.normalize();

            // Convert ray to world (Left camera) coordinates
            cv::Mat R_double;
            calib_.R.convertTo(R_double, CV_64F);
            cv::Mat T_double;
            calib_.T.convertTo(T_double, CV_64F);

            Eigen::Matrix3d R_eigen;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    R_eigen(i, j) = R_double.at<double>(i, j);
                }
            }
            Eigen::Vector3d T_eigen(T_double.at<double>(0, 0), T_double.at<double>(1, 0), T_double.at<double>(2, 0));

            Eigen::Vector3d rayDir = R_eigen * rayCamR;
            Eigen::Vector3d O = T_eigen; // Optical center of right camera in world (Left camera) coords
            
            Eigen::Vector3d w_vec = O - ballCentroid3D;
            double dot = w_vec.dot(rayDir);
            double disc = dot * dot - (w_vec.squaredNorm() - ballRadius_ * ballRadius_);
            if (disc >= 0.0) {
                double t = -dot - std::sqrt(disc);
                Marker3D marker3D;
                marker3D.position = O + t * rayDir;
                marker3D.confidence = 0.5; // Low confidence
                marker3D.isStereo = false;
                markers3D.push_back(marker3D);
            }
        }

        ball3D.markers = markers3D;
        trajectory.push_back(ball3D);
    }

    return trajectory;
}
