#include "rover/localization/pose_estimator.hpp"
#include <algorithm>
#include <cmath>

namespace rover::localization {

void PoseEstimator::set_pose(double x, double y, double yaw) {
    current_pose_.x = x;
    current_pose_.y = y;
    current_pose_.yaw = yaw;
    persistent_yaw_ = yaw;
}

void PoseEstimator::update_slam_correction(double slam_x, double slam_y, double slam_yaw) {
    slam_yaw_ = slam_yaw;
    slam_valid_ = true;
    
    current_pose_.x = 0.8 * current_pose_.x + 0.2 * slam_x;
    current_pose_.y = 0.8 * current_pose_.y + 0.2 * slam_y;
}

double PoseEstimator::fuse_heading(const RoverState& state, double dt) {
    // Default to whatever yaw was just calculated by dead reckoning/kinematics
    double candidate_yaw = persistent_yaw_;

    // 1. SLAM Heading (highest priority when active)
    if (slam_valid_) {
        candidate_yaw = slam_yaw_;
    } 
    // 2. Successive GNSS displacement vectors when moving
    else if (state.gps.rtk_fixed) {
        double dx = state.pose.x - last_gnss_x_;
        double dy = state.pose.y - last_gnss_y_;
        if ((dx * dx + dy * dy) > MIN_MOTION_DIST_SQ) {
            candidate_yaw = std::atan2(dy, dx);
            last_gnss_x_ = state.pose.x;
            last_gnss_y_ = state.pose.y;
        }
    } 
    // 3. Real Hardware IMU Heading (Only used if COM4 / hardware streams valid IMU data)
    else if (state.imu.valid) {
        candidate_yaw = state.imu.yaw;
    }
    // If none of these are active (e.g., pure simulation), it keeps the kinematic turn!

    persistent_yaw_ = candidate_yaw;
    return candidate_yaw;
}

void PoseEstimator::update(RoverState& state, double dt) {
    if (state.gps.rtk_fixed) {
        current_pose_.x = state.pose.x;
        current_pose_.y = state.pose.y;
    } else {
        // Kinematic Simulation Fallback:
        double v = state.velocity.linear;
        double w = state.velocity.angular;

        current_pose_.yaw += w * dt;
        
        while (current_pose_.yaw > M_PI)  current_pose_.yaw -= 2.0 * M_PI;
        while (current_pose_.yaw < -M_PI) current_pose_.yaw += 2.0 * M_PI;

        current_pose_.x += v * std::cos(current_pose_.yaw) * dt;
        current_pose_.y += v * std::sin(current_pose_.yaw) * dt;
    }

    // Feed current kinematic yaw into persistent storage before checking external sensors
    persistent_yaw_ = current_pose_.yaw;

    // Fuse heading (ignores IMU if state.imu.valid is false in simulation)
    current_pose_.yaw = fuse_heading(state, dt);
    state.pose = current_pose_;
}

} // namespace rover::localization