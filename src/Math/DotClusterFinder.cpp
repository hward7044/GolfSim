#include "Math/DotClusterFinder.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

// =============================================================================
// DotClusterConfig
// =============================================================================

DotClusterConfig DotClusterConfig::fromJson(const nlohmann::json& j, const DotClusterConfig& base) {
    DotClusterConfig cfg = base;
    if (!j.is_object()) return cfg;
    cfg.intensityThreshold = j.value("intensityThreshold", cfg.intensityThreshold);
    cfg.minDotArea         = j.value("minDotArea",         cfg.minDotArea);
    cfg.maxDotArea         = j.value("maxDotArea",         cfg.maxDotArea);
    cfg.maxDotAspect       = j.value("maxDotAspect",       cfg.maxDotAspect);
    cfg.clusterRadiusPx    = j.value("clusterRadiusPx",    cfg.clusterRadiusPx);
    cfg.minDotsPerCluster  = j.value("minDotsPerCluster",  cfg.minDotsPerCluster);
    cfg.maxDotsPerCluster  = j.value("maxDotsPerCluster",  cfg.maxDotsPerCluster);
    cfg.maxClusterSpreadPx = j.value("maxClusterSpreadPx", cfg.maxClusterSpreadPx);
    cfg.minClusterSpreadPx = j.value("minClusterSpreadPx", cfg.minClusterSpreadPx);
    cfg.intensityThreshold = std::clamp(cfg.intensityThreshold, 1, 254);
    return cfg;
}

nlohmann::json DotClusterConfig::toJson() const {
    return {
        {"intensityThreshold", intensityThreshold},
        {"minDotArea",         minDotArea},
        {"maxDotArea",         maxDotArea},
        {"maxDotAspect",       maxDotAspect},
        {"clusterRadiusPx",    clusterRadiusPx},
        {"minDotsPerCluster",  minDotsPerCluster},
        {"maxDotsPerCluster",  maxDotsPerCluster},
        {"maxClusterSpreadPx", maxClusterSpreadPx},
        {"minClusterSpreadPx", minClusterSpreadPx},
    };
}

// =============================================================================
// DotClusterResult
// =============================================================================

void DotClusterResult::clear() {
    clusters.clear();
    rejectedDots.clear();
    rejectedClusters.clear();
    threshold = 0;
}

static nlohmann::json clusterToJson(const DotCluster& c, bool accepted, const std::string& reason) {
    nlohmann::json markers = nlohmann::json::array();
    for (const auto& d : c.dots) {
        markers.push_back({d.centroid.x, d.centroid.y});
    }
    return {
        {"centroid",      {c.centroid.x, c.centroid.y}},
        {"boundingBox",   {c.boundingBox.x, c.boundingBox.y, c.boundingBox.width, c.boundingBox.height}},
        {"area",          c.totalArea},
        {"circularity",   1.0},
        {"isOverlapping", false},
        {"accepted",      accepted},
        {"reason",        reason},
        {"dotCount",      c.dots.size()},
        {"spreadPx",      c.spreadPx},
        {"markers",       markers},
    };
}

nlohmann::json DotClusterResult::toJson(std::size_t maxRejectedDots) const {
    nlohmann::json candidates = nlohmann::json::array();
    for (const auto& c : clusters) {
        candidates.push_back(clusterToJson(c, true, "Accepted (DotCluster: " + std::to_string(c.dots.size()) + " dots)"));
    }
    for (const auto& r : rejectedClusters) {
        candidates.push_back(clusterToJson(r.cluster, false, r.reason));
    }

    nlohmann::json dots = nlohmann::json::array();
    for (const auto& c : clusters) {
        for (const auto& d : c.dots) {
            dots.push_back({{"centroid", {d.centroid.x, d.centroid.y}}, {"area", d.area},
                            {"peak", d.peak}, {"accepted", true}});
        }
    }
    std::size_t shown = 0;
    for (const auto& r : rejectedDots) {
        if (shown++ >= maxRejectedDots) break;
        dots.push_back({{"centroid", {r.dot.centroid.x, r.dot.centroid.y}}, {"area", r.dot.area},
                        {"peak", r.dot.peak}, {"accepted", false}, {"reason", r.reason},
                        {"boundingBox", {r.dot.box.x, r.dot.box.y, r.dot.box.width, r.dot.box.height}}});
    }

    return {
        {"candidates",       candidates},
        {"dots",             dots},
        {"threshold",        threshold},
        {"rejectedDotCount", rejectedDots.size()},
    };
}

// =============================================================================
// DotClusterFinder
// =============================================================================

DotClusterFinder::DotClusterFinder(DotClusterConfig config, double nominalBallRadiusPx)
    : config_(config), ballRadiusPx_(nominalBallRadiusPx) {}

void DotClusterFinder::find(const cv::Mat& frame, DotClusterResult& out) {
    out.clear();
    out.threshold = config_.intensityThreshold;
    if (frame.empty()) return;

    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray_, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray_, cv::COLOR_BGRA2GRAY);
    } else {
        gray_ = frame;
    }

    // 1. Binarise: the "intensity" setting. Everything above is a candidate dot.
    cv::threshold(gray_, mask_, config_.intensityThreshold, 255, cv::THRESH_BINARY);

    // 2. Dots, 3-5. Clusters
    extractDots(out);
    clusterDots(out);
}

void DotClusterFinder::extractDots(DotClusterResult& out) {
    contours_.clear();
    dots_.clear();
    cv::findContours(mask_, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours_) {
        Dot dot;
        dot.box = cv::boundingRect(contour);
        // A 1-2 px glint has zero polygon area but is a real dot: fall back to
        // the pixel count of its bounding box.
        dot.area = cv::contourArea(contour);
        if (dot.area < 1.0) {
            dot.area = static_cast<double>(cv::countNonZero(mask_(dot.box)));
        }

        cv::Moments m = cv::moments(contour);
        if (m.m00 > 0.0) {
            dot.centroid = cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
        } else {
            dot.centroid = cv::Point2d(dot.box.x + dot.box.width / 2.0,
                                       dot.box.y + dot.box.height / 2.0);
        }

        double maxVal = 0.0;
        cv::minMaxLoc(gray_(dot.box), nullptr, &maxVal);
        dot.peak = maxVal;

        const double longSide  = std::max(dot.box.width, dot.box.height);
        const double shortSide = std::max(1, std::min(dot.box.width, dot.box.height));
        const double aspect    = longSide / shortSide;

        if (dot.area < config_.minDotArea) {
            out.rejectedDots.push_back({dot, "Dot: area too small"});
        } else if (dot.area > config_.maxDotArea) {
            out.rejectedDots.push_back({dot, "Dot: area too large"});
        } else if (aspect > config_.maxDotAspect) {
            out.rejectedDots.push_back({dot, "Dot: elongated (aspect)"});
        } else {
            dots_.push_back(dot);
        }
    }
}

void DotClusterFinder::clusterDots(DotClusterResult& out) {
    const std::size_t n = dots_.size();
    assigned_.assign(n, 0);
    const double radius2 = config_.clusterRadiusPx * config_.clusterRadiusPx;

    for (std::size_t seed = 0; seed < n; ++seed) {
        if (assigned_[seed]) continue;

        DotCluster cluster;
        cluster.dots.push_back(dots_[seed]);
        assigned_[seed] = 1;
        cv::Point2d centre = dots_[seed].centroid;

        // Grow until no unassigned dot lies within clusterRadiusPx of the
        // running centroid. Bounded by the dot count, which is tiny on a
        // dots-only frame.
        bool grew = true;
        while (grew) {
            grew = false;
            for (std::size_t j = 0; j < n; ++j) {
                if (assigned_[j]) continue;
                const cv::Point2d d = dots_[j].centroid - centre;
                if (d.x * d.x + d.y * d.y <= radius2) {
                    cluster.dots.push_back(dots_[j]);
                    assigned_[j] = 1;
                    grew = true;
                    cv::Point2d sum(0.0, 0.0);
                    for (const auto& dot : cluster.dots) sum += dot.centroid;
                    centre = sum * (1.0 / static_cast<double>(cluster.dots.size()));
                }
            }
        }

        // Intensity-weighted centroid: brighter glints are better localised.
        double weight = 0.0;
        cv::Point2d weighted(0.0, 0.0);
        cluster.totalArea = 0.0;
        for (const auto& dot : cluster.dots) {
            const double w = std::max(dot.peak, 1.0);
            weighted += dot.centroid * w;
            weight += w;
            cluster.totalArea += dot.area;
        }
        cluster.centroid = weighted * (1.0 / weight);

        double maxDist2 = 0.0;
        for (const auto& dot : cluster.dots) {
            const cv::Point2d d = dot.centroid - cluster.centroid;
            maxDist2 = std::max(maxDist2, d.x * d.x + d.y * d.y);
        }
        cluster.spreadPx = 2.0 * std::sqrt(maxDist2);

        const int r = static_cast<int>(std::lround(ballRadiusPx_));
        cluster.boundingBox = cv::Rect(static_cast<int>(std::lround(cluster.centroid.x)) - r,
                                       static_cast<int>(std::lround(cluster.centroid.y)) - r,
                                       2 * r, 2 * r) & cv::Rect(0, 0, gray_.cols, gray_.rows);

        const int count = static_cast<int>(cluster.dots.size());
        if (count < config_.minDotsPerCluster) {
            out.rejectedClusters.push_back({std::move(cluster), "Cluster: too few dots"});
        } else if (count > config_.maxDotsPerCluster) {
            out.rejectedClusters.push_back({std::move(cluster), "Cluster: too many dots"});
        } else if (cluster.spreadPx > config_.maxClusterSpreadPx) {
            out.rejectedClusters.push_back({std::move(cluster), "Cluster: spread too large"});
        } else if (cluster.spreadPx < config_.minClusterSpreadPx) {
            out.rejectedClusters.push_back({std::move(cluster), "Cluster: spread too small"});
        } else {
            out.clusters.push_back(std::move(cluster));
        }
    }

    std::sort(out.clusters.begin(), out.clusters.end(),
              [](const DotCluster& a, const DotCluster& b) { return a.dots.size() > b.dots.size(); });
}
