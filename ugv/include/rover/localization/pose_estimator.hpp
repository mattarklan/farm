#pragma once

#include "rover/common/types.hpp"
#include "rover/state/rover_state.hpp"
#include <cmath>

namespace rover::localization {

class PoseEstimator {
public:
    PoseEstimator() = default;

    // Direct initialization/reset of persistent pose
    void set_pose(double x, double y, double yaw);

    // Primary update function called every tick (e.g., 20 Hz)
    void update(RoverState& state, double dt);

    // Inject relative SLAM/Lidar scan-matching corrections when available
    void update_slam_correction(double slam_x, double slam_y, double slam_yaw);

private:
    double fuse_heading(const RoverState& state, double dt);

    Pose current_pose_{0.0, 0.0, 0.0};
    
    // Heading tracking variables
    double persistent_yaw_{0.0};
    double slam_yaw_{0.0};
    bool slam_valid_{false};

    // Tracking displacement for GNSS-derived motion heading
    double last_gnss_x_{0.0};
    double last_gnss_y_{0.0};
    static constexpr double MIN_MOTION_DIST_SQ = 0.04; // 0.2m movement threshold
};

} // namespace rover::localization