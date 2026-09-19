#include "Math/DotClusterTracker.hpp"

DotClusterTracker::DotClusterTracker(DotClusterConfig config, double nominalBallRadiusPx)
    : finder_(config, nominalBallRadiusPx) {
    latestDiag_ = {{"candidates", nlohmann::json::array()}};
}

std::vector<BallObservation> DotClusterTracker::detectBalls(const cv::Mat& frame) {
    std::vector<BallObservation> observations;
    finder_.find(frame, result_);

    observations.reserve(result_.clusters.size());
    for (const auto& cluster : result_.clusters) {
        BallObservation obs;
        obs.centroid = cluster.centroid;
        obs.boundingBox = cluster.boundingBox;
        obs.markers.reserve(cluster.dots.size());
        for (const auto& dot : cluster.dots) {
            MarkerObservation marker;
            marker.position = dot.centroid;
            marker.intensity = dot.peak;
            obs.markers.push_back(marker);
        }
        observations.push_back(std::move(obs));
    }

    latestDiag_ = result_.toJson();
    return observations;
}
