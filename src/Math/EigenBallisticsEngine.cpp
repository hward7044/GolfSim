#include "Math/EigenBallisticsEngine.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <numbers>


struct LocalMarker {
    Eigen::Vector3d vec;
    double conf;
};

// Forward declaration of helpers
static bool computeProcrustes(
    const std::vector<LocalMarker>& locP,
    const std::vector<LocalMarker>& locQ,
    const std::vector<int>& p_idx,
    const std::vector<int>& q_idx,
    Eigen::Matrix3d& R,
    double& weight,
    double& residual
) {
    int m = p_idx.size();
    if (m == 2) {
        Eigen::Vector3d p1 = locP[p_idx[0]].vec;
        Eigen::Vector3d p2 = locP[p_idx[1]].vec;
        Eigen::Vector3d q1 = locQ[q_idx[0]].vec;
        Eigen::Vector3d q2 = locQ[q_idx[1]].vec;

        double norm_p1 = p1.norm();
        double norm_p2 = p2.norm();
        double norm_q1 = q1.norm();
        double norm_q2 = q2.norm();
        if (norm_p1 < 1e-4 || norm_p2 < 1e-4 || norm_q1 < 1e-4 || norm_q2 < 1e-4) {
            return false;
        }

        p1.normalize();
        p2.normalize();
        q1.normalize();
        q2.normalize();

        // Check for collinearity degeneracy
        double dot_p = std::abs(p1.dot(p2));
        double dot_q = std::abs(q1.dot(q2));
        if (dot_p > 0.98 || dot_q > 0.98) {
            return false;
        }

        // Synthesize third orthogonal vector (TRIAD method refinement)
        Eigen::Vector3d p3 = p1.cross(p2);
        p3.normalize();
        Eigen::Vector3d q3 = q1.cross(q2);
        q3.normalize();

        // Build normalized cross-covariance matrix H
        Eigen::Matrix3d H = p1 * q1.transpose() + p2 * q2.transpose() + p3 * q3.transpose();

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        R = V * U.transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
            S(2, 2) = -1.0;
            R = V * S * U.transpose();
        }

        residual = (R * p1 - q1).squaredNorm() + (R * p2 - q2).squaredNorm();
        double c1 = locP[p_idx[0]].conf * locQ[q_idx[0]].conf;
        double c2 = locP[p_idx[1]].conf * locQ[q_idx[1]].conf;
        weight = 0.5 * (c1 + c2) * 0.8; // 20% penalty for synthesized vector
        return true;
    } else {
        // m >= 3 case
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        std::vector<Eigen::Vector3d> normP(m), normQ(m);
        for (int i = 0; i < m; ++i) {
            Eigen::Vector3d p = locP[p_idx[i]].vec;
            Eigen::Vector3d q = locQ[q_idx[i]].vec;
            double np = p.norm();
            double nq = q.norm();
            if (np < 1e-4 || nq < 1e-4) return false;
            p.normalize();
            q.normalize();
            normP[i] = p;
            normQ[i] = q;
            H += p * q.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        R = V * U.transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
            S(2, 2) = -1.0;
            R = V * S * U.transpose();
        }

        residual = 0.0;
        double confSum = 0.0;
        for (int i = 0; i < m; ++i) {
            residual += (R * normP[i] - normQ[i]).squaredNorm();
            confSum += locP[p_idx[i]].conf * locQ[q_idx[i]].conf;
        }
        weight = confSum / m;
        return true;
    }
}

static bool findBestRotation(
    const std::vector<Marker3D>& P,
    const std::vector<Marker3D>& Q,
    const Eigen::Vector3d& C_P,
    const Eigen::Vector3d& C_Q,
    Eigen::Matrix3d& bestR,
    double& bestWeight
) {
    std::vector<LocalMarker> locP, locQ;
    for (const auto& m : P) {
        locP.push_back({ m.position - C_P, m.confidence });
    }
    for (const auto& m : Q) {
        locQ.push_back({ m.position - C_Q, m.confidence });
    }

    int np = static_cast<int>(locP.size());
    int nq = static_cast<int>(locQ.size());
    if (np < 2 || nq < 2) return false;

    int bestM = 0;
    double minResidual = 1e9;
    Eigen::Matrix3d finalR = Eigen::Matrix3d::Identity();
    double finalWeight = 0.0;

    int max_m = std::min(np, nq);
    for (int m_size = max_m; m_size >= 2; --m_size) {
        if (m_size == 4) {
            std::vector<int> p_idx = {0, 1, 2, 3};
            std::vector<int> q_idx = {0, 1, 2, 3};
            do {
                Eigen::Matrix3d R;
                double weight = 0.0;
                double residual = 0.0;
                if (computeProcrustes(locP, locQ, p_idx, q_idx, R, weight, residual)) {
                    if (residual < minResidual && residual < 0.05) {
                        minResidual = residual;
                        bestM = 4;
                        finalR = R;
                        finalWeight = weight;
                    }
                }
            } while (std::next_permutation(q_idx.begin(), q_idx.end()));
        } else if (m_size == 3) {
            for (int p0 = 0; p0 < np; ++p0) {
                for (int p1 = p0 + 1; p1 < np; ++p1) {
                    for (int p2 = p1 + 1; p2 < np; ++p2) {
                        std::vector<int> p_idx = {p0, p1, p2};
                        for (int q0 = 0; q0 < nq; ++q0) {
                            for (int q1 = 0; q1 < nq; ++q1) {
                                if (q1 == q0) continue;
                                for (int q2 = 0; q2 < nq; ++q2) {
                                    if (q2 == q0 || q2 == q1) continue;
                                    std::vector<int> q_idx = {q0, q1, q2};
                                    Eigen::Matrix3d R;
                                    double weight = 0.0;
                                    double residual = 0.0;
                                    if (computeProcrustes(locP, locQ, p_idx, q_idx, R, weight, residual)) {
                                        if (residual < minResidual && residual < 0.05) {
                                            minResidual = residual;
                                            bestM = 3;
                                            finalR = R;
                                            finalWeight = weight;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else if (m_size == 2) {
            for (int p0 = 0; p0 < np; ++p0) {
                for (int p1 = p0 + 1; p1 < np; ++p1) {
                    std::vector<int> p_idx = {p0, p1};
                    for (int q0 = 0; q0 < nq; ++q0) {
                        for (int q1 = 0; q1 < nq; ++q1) {
                            if (q1 == q0) continue;
                            std::vector<int> q_idx = {q0, q1};
                            Eigen::Matrix3d R;
                            double weight = 0.0;
                            double residual = 0.0;
                            if (computeProcrustes(locP, locQ, p_idx, q_idx, R, weight, residual)) {
                                if (residual < minResidual && residual < 0.05) {
                                    minResidual = residual;
                                    bestM = 2;
                                    finalR = R;
                                    finalWeight = weight;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (bestM > 0) {
            bestR = finalR;
            bestWeight = finalWeight;
            return true;
        }
    }
    return false;
}

static double weightedMedian(std::vector<std::pair<double, double>>& values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    double totalWeight = 0.0;
    for (const auto& p : values) totalWeight += p.second;
    if (totalWeight <= 0.0) return values[values.size() / 2].first;
    
    double weightSum = 0.0;
    for (const auto& p : values) {
        weightSum += p.second;
        if (weightSum >= 0.5 * totalWeight) {
            return p.first;
        }
    }
    return values.back().first;
}

LaunchData<Degrees, MilesPerHour> EigenBallisticsEngine::solveKinematics(
    const std::vector<Ball3D>& trajectory,
    double pulseIntervalMs
) {
    LaunchData<Degrees, MilesPerHour> out;
    out.ballSpeed = MilesPerHour(0.0);
    out.verticalLaunchAngle = Degrees(0.0);
    out.horizontalLaunchAngle = Degrees(0.0);
    out.spinRPM = 0.0;
    out.spinAxis = Eigen::Vector3d::Zero();

    std::size_t N = trajectory.size();
    if (N < 2) {
        return out;
    }

    double dt = pulseIntervalMs / 1000.0;

    // 1. Solve Trajectory Linear Regression for Velocity
    Eigen::MatrixXd A(N, 2);
    Eigen::VectorXd b_x(N), b_y(N), b_z(N);
    for (std::size_t k = 0; k < N; ++k) {
        double tk = k * dt;
        A(k, 0) = tk;
        A(k, 1) = 1.0;
        b_x(k) = trajectory[k].centroid.x();
        b_y(k) = trajectory[k].centroid.y();
        b_z(k) = trajectory[k].centroid.z();
    }

    Eigen::Vector2d res_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::Vector2d res_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::Vector2d res_z = A.colPivHouseholderQr().solve(b_z);

    double vx = res_x(0);
    double vy = res_y(0);
    double vz = res_z(0);

    double speed_mps = std::sqrt(vx*vx + vy*vy + vz*vz);
    out.ballSpeed = to_mph(MetersPerSecond(speed_mps));

    // Coordinate transformation: 
    // Camera Frame: X = downrange, Y = vertical (downwards), Z = depth (lateral)
    // World Frame: X_w = X, Y_w = -Y (vertical up), Z_w = Z (lateral)
    double vla_rad = std::atan2(-vy, std::sqrt(vx*vx + vz*vz));
    out.verticalLaunchAngle = to_degrees(Radians(vla_rad));

    double hla_rad = std::atan2(vz, vx);
    out.horizontalLaunchAngle = to_degrees(Radians(hla_rad));

    // 2. Solve Spin using Robust R^3 Angular Velocity Averaging
    std::vector<Eigen::Vector3d> omegas;
    std::vector<double> weights;

    for (std::size_t k = 0; k < N - 1; ++k) {
        Eigen::Matrix3d R;
        double w = 0.0;
        if (findBestRotation(trajectory[k].markers, trajectory[k+1].markers, trajectory[k].centroid, trajectory[k+1].centroid, R, w)) {
            Eigen::AngleAxisd angleAxis(R);
            double angle = angleAxis.angle();
            Eigen::Vector3d axis = angleAxis.axis();
            
            // Angular velocity in rad/s
            Eigen::Vector3d omega = (angle / dt) * axis;
            omegas.push_back(omega);
            weights.push_back(w);
        }
    }

    if (!omegas.empty()) {
        std::vector<std::pair<double, double>> wx, wy, wz;
        for (std::size_t i = 0; i < omegas.size(); ++i) {
            wx.push_back({ omegas[i].x(), weights[i] });
            wy.push_back({ omegas[i].y(), weights[i] });
            wz.push_back({ omegas[i].z(), weights[i] });
        }

        double avg_wx = weightedMedian(wx);
        double avg_wy = weightedMedian(wy);
        double avg_wz = weightedMedian(wz);

        Eigen::Vector3d omega_avg(avg_wx, avg_wy, avg_wz);
        double omega_norm = omega_avg.norm();
        if (omega_norm > 1e-4) {
            out.spinRPM = omega_norm * 60.0 / (2.0 * std::numbers::pi);
            // Transform spin axis to world frame (negate Y axis component)
            out.spinAxis = Eigen::Vector3d(omega_avg.x(), -omega_avg.y(), omega_avg.z()).normalized();
        }
    }

    return out;
}
