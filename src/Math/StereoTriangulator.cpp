#include "Math/StereoTriangulator.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/stereo.hpp>
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

void StereoTriangulator::updateExtrinsics() {
    if (!calib_.R.empty() && !calib_.T.empty()) {
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
        Eigen::Vector3d T_eigen(
            T_double.at<double>(0, 0),
            T_double.at<double>(1, 0),
            T_double.at<double>(2, 0)
        );

        // Right-to-world (Left camera) rotation: R_inv = R^T
        R_inv_ = R_eigen.transpose();
        // Right camera optical center in world (Left camera) coords: O_R = -R^T * T
        rightOpticalCenter_ = -R_inv_ * T_eigen;
    }
}

// Sorts 2D observation indices along the principal motion axis using covariance analysis.
// Orients the vector in the flight direction (+X default, with -Y upwards tie-break for steep launches).
static std::vector<std::size_t> sortAlongPrincipalAxis(
    const std::vector<cv::Point2d>& pts,
    std::vector<double>& outProjections,
    double flightDirectionX = 1.0
) {
    std::size_t n = pts.size();
    std::vector<std::size_t> indices(n);
    for (std::size_t i = 0; i < n; ++i) indices[i] = i;
    outProjections.assign(n, 0.0);

    if (n < 2) {
        return indices;
    }

    // Compute centroid
    double meanX = 0.0;
    double meanY = 0.0;
    for (const auto& pt : pts) {
        meanX += pt.x;
        meanY += pt.y;
    }
    meanX /= static_cast<double>(n);
    meanY /= static_cast<double>(n);

    // Compute 2x2 covariance matrix
    double cxx = 0.0;
    double cyy = 0.0;
    double cxy = 0.0;
    for (const auto& pt : pts) {
        double dx = pt.x - meanX;
        double dy = pt.y - meanY;
        cxx += dx * dx;
        cyy += dy * dy;
        cxy += dx * dy;
    }
    cxx /= static_cast<double>(n);
    cyy /= static_cast<double>(n);
    cxy /= static_cast<double>(n);

    // Compute principal eigenvector analytically for 2x2 symmetric matrix
    double trace = cxx + cyy;
    double det = cxx * cyy - cxy * cxy;
    double disc = std::max(0.0, trace * trace - 4.0 * det);
    double lambda1 = (trace + std::sqrt(disc)) * 0.5;

    double vx = 1.0;
    double vy = 0.0;

    if (std::abs(cxy) > 1e-9) {
        vx = lambda1 - cyy;
        vy = cxy;
    } else {
        if (cyy > cxx) {
            vx = 0.0;
            vy = 1.0;
        } else {
            vx = 1.0;
            vy = 0.0;
        }
    }

    double norm = std::hypot(vx, vy);
    if (norm > 1e-9) {
        vx /= norm;
        vy /= norm;
    } else {
        vx = (flightDirectionX >= 0.0) ? 1.0 : -1.0;
        vy = 0.0;
    }

    // Orient principal vector downrange in flight direction:
    // In camera image space:
    // - Horizontal flight is along +X (if flightDirectionX > 0) or -X (if flightDirectionX < 0)
    // - Upward launch into the air corresponds to decreasing Y (vy < 0 in OpenCV image coordinates)
    double targetSignX = (flightDirectionX >= 0.0) ? 1.0 : -1.0;
    if (std::abs(vx) > 0.1) {
        if ((vx * targetSignX) < 0.0) {
            vx = -vx;
            vy = -vy;
        }
    } else {
        // Steep vertical launch (e.g. lob wedge): horizontal delta is near zero
        // In image pixels, upward motion into the air has decreasing Y (vy < 0)
        if (vy > 0.0) {
            vx = -vx;
            vy = -vy;
        }
    }

    // Project points onto the oriented principal vector: s = (p - mean) . v
    for (std::size_t i = 0; i < n; ++i) {
        outProjections[i] = (pts[i].x - meanX) * vx + (pts[i].y - meanY) * vy;
    }

    std::sort(indices.begin(), indices.end(), [&](std::size_t a, std::size_t b) {
        return outProjections[a] < outProjections[b];
    });

    return indices;
}

StereoTriangulator::StereoTriangulator() : ballRadius_(0.021335) {
    // Default calibration parameters mapping to a horizontal stereo setup with 100mm baseline
    calib_.K_L = cv::Mat_<double>({3, 3}, {1000.0, 0.0, 640.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 1.0});
    calib_.D_L = cv::Mat::zeros(1, 5, CV_64F);
    
    calib_.K_R = calib_.K_L.clone();
    calib_.D_R = calib_.D_L.clone();

    calib_.R = cv::Mat::eye(3, 3, CV_64F);
    calib_.T = cv::Mat_<double>({3, 1}, {-0.1, 0.0, 0.0}); // 100mm baseline along X

    calib_.R_L = cv::Mat::eye(3, 3, CV_64F);
    calib_.R_R = cv::Mat::eye(3, 3, CV_64F);

    // Identity projection matrices
    calib_.P_L = cv::Mat_<double>({3, 4}, {1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0});
    calib_.P_R = cv::Mat_<double>({3, 4}, {1000.0, 0.0, 640.0, -100.0, 0.0, 1000.0, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0});

    updateExtrinsics();
}

StereoTriangulator::StereoTriangulator(const StereoCalibration& calib, double ballRadius)
    : calib_(calib), ballRadius_(ballRadius) {
    updateExtrinsics();
}

void StereoTriangulator::setCalibration(const StereoCalibration& calib) {
    calib_ = calib;
    updateExtrinsics();
}

std::vector<Ball3D> StereoTriangulator::triangulateShot(
    const std::vector<BallObservation>& leftObs,
    const std::vector<BallObservation>& rightObs
) {
    std::vector<Ball3D> trajectory;

    // 1. Rectify and project all ball centroids to rectified space via OpenCV 5 batch SIMD
    std::vector<cv::Point2d> rectLeft;
    if (!leftObs.empty()) {
        std::vector<cv::Point2d> leftPts(leftObs.size());
        for (std::size_t i = 0; i < leftObs.size(); ++i) {
            leftPts[i] = leftObs[i].centroid;
        }
        cv::undistortPoints(leftPts, rectLeft, calib_.K_L, calib_.D_L, calib_.R_L, calib_.P_L);
    }

    std::vector<cv::Point2d> rectRight;
    if (!rightObs.empty()) {
        std::vector<cv::Point2d> rightPts(rightObs.size());
        for (std::size_t j = 0; j < rightObs.size(); ++j) {
            rightPts[j] = rightObs[j].centroid;
        }
        cv::undistortPoints(rightPts, rectRight, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);
    }

    // 2. Sort indices along principal motion axis to preserve chronological path structure
    std::vector<double> leftProjections;
    std::vector<std::size_t> sortedLeftIdx = sortAlongPrincipalAxis(rectLeft, leftProjections, flightDirectionX_);

    std::vector<double> rightProjections;
    std::vector<std::size_t> sortedRightIdx = sortAlongPrincipalAxis(rectRight, rightProjections, flightDirectionX_);

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

    // Sort matches chronologically based on Left observation projection along principal flight axis
    std::sort(matches.begin(), matches.end(), [&](const auto& a, const auto& b) {
        return leftProjections[a.first] < leftProjections[b.first];
    });

    // 4. Triangulate the matched pairs
    for (const auto& match : matches) {
        const auto& bL = leftObs[match.first];
        const auto& bR = rightObs[match.second];

        // Triangulate ball centroid using already-rectified coordinates
        pt2D_L_.create(1, 1, CV_64FC2);
        pt2D_L_.at<cv::Vec2d>(0, 0) = cv::Vec2d(rectLeft[match.first].x, rectLeft[match.first].y);
        pt2D_R_.create(1, 1, CV_64FC2);
        pt2D_R_.at<cv::Vec2d>(0, 0) = cv::Vec2d(rectRight[match.second].x, rectRight[match.second].y);

        cv::triangulatePoints(calib_.P_L, calib_.P_R, pt2D_L_, pt2D_R_, pt4D_);
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

        // Prep Right camera marker undistorted rectified positions for epipolar matching via batch SIMD
        std::vector<cv::Point2d> undMrList;
        if (!bR.markers.empty()) {
            std::vector<cv::Point2d> mrPts(bR.markers.size());
            for (std::size_t k = 0; k < bR.markers.size(); ++k) mrPts[k] = bR.markers[k].position;
            cv::undistortPoints(mrPts, undMrList, calib_.K_R, calib_.D_R, calib_.R_R, calib_.P_R);
        }

        // Prep Left camera marker undistorted rectified positions
        std::vector<cv::Point2d> undMlList;
        if (!bL.markers.empty()) {
            std::vector<cv::Point2d> mlPts(bL.markers.size());
            for (std::size_t k = 0; k < bL.markers.size(); ++k) mlPts[k] = bL.markers[k].position;
            cv::undistortPoints(mlPts, undMlList, calib_.K_L, calib_.D_L, calib_.R_L, calib_.P_L);
        }

        // Prep Left camera marker normalized coordinates for ray-sphere fallback
        std::vector<cv::Point2d> normMlList;
        if (!bL.markers.empty()) {
            std::vector<cv::Point2d> mlPts(bL.markers.size());
            for (std::size_t k = 0; k < bL.markers.size(); ++k) mlPts[k] = bL.markers[k].position;
            cv::undistortPoints(mlPts, normMlList, calib_.K_L, calib_.D_L);
        }

        // Try to match Left markers with Right markers
        for (std::size_t lIdx = 0; lIdx < bL.markers.size(); ++lIdx) {
            cv::Point2d undMl = undMlList[lIdx];

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
                cv::Point2d normL = normMlList[lIdx];
                double x_norm = normL.x;
                double y_norm = normL.y;
                
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
        std::vector<cv::Point2d> normMrList;
        if (!bR.markers.empty()) {
            std::vector<cv::Point2d> mrPts(bR.markers.size());
            for (std::size_t k = 0; k < bR.markers.size(); ++k) mrPts[k] = bR.markers[k].position;
            cv::undistortPoints(mrPts, normMrList, calib_.K_R, calib_.D_R);
        }

        for (std::size_t r = 0; r < bR.markers.size(); ++r) {
            if (rightMarkerUsed[r]) continue;

            cv::Point2d normR = normMrList[r];
            double x_norm = normR.x;
            double y_norm = normR.y;

            // Ray in Right camera frame
            Eigen::Vector3d rayCamR(x_norm, y_norm, 1.0);
            rayCamR.normalize();

            // Transform ray to world (Left camera) coordinates using precomputed extrinsics
            Eigen::Vector3d rayDir = (R_inv_ * rayCamR).normalized();
            Eigen::Vector3d O = rightOpticalCenter_; // Exact optical center of right camera in world coords
            
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
