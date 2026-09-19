#pragma once
#include "Math/IComputerVision.hpp"
#include "Math/DotClusterFinder.hpp"
#include "Diagnostics/IDiagnosticProvider.hpp"
#include <opencv2/core.hpp>
#include <vector>

/**
 * @brief IComputerVision over DotClusterFinder: each accepted dot cluster is one
 * ball, and its dots are the ball's markers (there is no separate marker pass —
 * the glints the spin solver wants are exactly the dots that located the ball).
 *
 * Replaces OpenCVMomentsTracker, which looked for a filled circular silhouette
 * and therefore could not see a correctly exposed dots-only ball at all.
 */
class DotClusterTracker : public IComputerVision, public IDiagnosticProvider {
private:
    DotClusterFinder finder_;
    DotClusterResult result_;
    nlohmann::json   latestDiag_;

public:
    explicit DotClusterTracker(DotClusterConfig config = DotClusterConfig(),
                               double nominalBallRadiusPx = 23.3);

    std::vector<BallObservation> detectBalls(const cv::Mat& frame) override;

    nlohmann::json getLatestDiagnostics() const override { return latestDiag_; }

    const DotClusterConfig& config() const noexcept { return finder_.config(); }
    void setConfig(const DotClusterConfig& config) { finder_.setConfig(config); }
};
