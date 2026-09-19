#pragma once
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <vector>

/**
 * @brief Tunables for dots-only ball detection.
 *
 * The launch monitor images the ball as a cluster of retroreflective glints on
 * a black field, not as a silhouette. Measured at the 3.0 ft working distance
 * (docs/refactor/09, section 1.1): the lit cap is ~20-22 px across, about
 * 0.45x the nominal 46.7 px ball diameter, holding 4-10 dots of 3-10 px each.
 */
struct DotClusterConfig {
    int    intensityThreshold  = 135;   ///< The "intensity" setting: pixel > this is 1/white, else 0/black
    double minDotArea          = 2.0;   ///< px^2; below this is sensor noise
    double maxDotArea          = 120.0; ///< px^2; above this is a bloomed/merged region or a reflection
    double maxDotAspect        = 2.5;   ///< Bounding-box aspect ratio; rejects elongated specular bars
    double clusterRadiusPx     = 28.0;  ///< Dots within this distance of a cluster centroid belong to it
    int    minDotsPerCluster   = 3;     ///< Fewer is a stray reflection, not a ball
    int    maxDotsPerCluster   = 40;    ///< More is a lit-up background
    double maxClusterSpreadPx  = 47.0;  ///< Max extent of one cluster: one ball diameter
    double minClusterSpreadPx  = 6.0;   ///< Min extent: a single bloomed dot is not a ball

    /// Merge fields present in `j` over `base`; absent fields keep base values.
    static DotClusterConfig fromJson(const nlohmann::json& j, const DotClusterConfig& base = DotClusterConfig());
    nlohmann::json toJson() const;
};

struct Dot {
    cv::Point2d centroid;   ///< Sub-pixel centroid from image moments
    double      area = 0.0; ///< px^2 (contour area, or pixel count for 1-2 px glints)
    double      peak = 0.0; ///< Brightest pixel inside the dot
    cv::Rect    box;        ///< Bounding box in frame coordinates
};

struct DotCluster {
    cv::Point2d      centroid;        ///< Intensity-weighted mean of the dot centroids
    double           spreadPx = 0.0;  ///< Twice the farthest dot-to-centroid distance
    double           totalArea = 0.0; ///< Sum of dot areas
    cv::Rect         boundingBox;     ///< Nominal ball box around the centroid, clipped to the frame
    std::vector<Dot> dots;
};

/// Everything one call to DotClusterFinder::find() produced, accepted or not.
/// Owned by the caller so its buffers are reused frame to frame.
struct DotClusterResult {
    struct RejectedDot     { Dot dot;            const char* reason = ""; };
    struct RejectedCluster { DotCluster cluster; const char* reason = ""; };

    std::vector<DotCluster>      clusters;          ///< Accepted balls, best (most dots) first
    std::vector<RejectedDot>     rejectedDots;
    std::vector<RejectedCluster> rejectedClusters;
    int threshold = 0;

    void clear();

    /// Diagnostics in the `candidates[]` shape FlightRecorder already draws,
    /// plus a `dots[]` list. Rejected dots are capped so JSON stays bounded.
    nlohmann::json toJson(std::size_t maxRejectedDots = 50) const;
};

/**
 * @brief Finds retroreflective-dot clusters (balls) in a grayscale frame.
 *
 * Threshold -> dot blobs (area / aspect gates) -> greedy radius clustering ->
 * cluster gates (count, spread) -> intensity-weighted centroid. Runs on the
 * whole frame: with continuous strobing nothing is time-critical before the
 * shot, and a mostly-black 1280x800 frame thresholds in well under a millisecond.
 */
class DotClusterFinder {
public:
    explicit DotClusterFinder(DotClusterConfig config = DotClusterConfig(),
                              double nominalBallRadiusPx = 23.3);

    void find(const cv::Mat& frame, DotClusterResult& out);

    const DotClusterConfig& config() const noexcept { return config_; }
    void setConfig(const DotClusterConfig& config) { config_ = config; }
    double nominalBallRadiusPx() const noexcept { return ballRadiusPx_; }

private:
    DotClusterConfig config_;
    double           ballRadiusPx_;

    // Scratch buffers reused across calls
    cv::Mat gray_;
    cv::Mat mask_;
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<Dot>  dots_;
    std::vector<char> assigned_;

    void extractDots(DotClusterResult& out);
    void clusterDots(DotClusterResult& out);
};
