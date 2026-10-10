#include "rover/planning/local_planner.hpp"
#include <cmath>
#include <algorithm>

namespace rover::planning {

LocalPlanner::LocalPlanner(const StanleyConfig& config) : config_(config) {}

Velocity LocalPlanner::compute_command(const Pose& current_pose, 
                                        const std::vector<Pose>& path, 
                                        size_t& target_idx) 
{
    Velocity cmd{0.0, 0.0};
    if (path.empty() || target_idx >= path.size()) {
        return cmd;
    }

    // 1. Advance target index if close to current waypoint
    const auto& target = path[target_idx];
    double dist_to_target = std::hypot(target.x - current_pose.x, target.y - current_pose.y);
    if (dist_to_target < config_.goal_tolerance && target_idx + 1 < path.size()) {
        target_idx++;
    }

    // Final waypoint goal check
    if (target_idx == path.size() - 1 && dist_to_target < config_.goal_tolerance) {
        return cmd; // Arrived at destination
    }

    // 2. Reference path segment orientation
    const auto& ref_pt = path[target_idx];
    double path_yaw = ref_pt.yaw;

    // 3. Calculate Cross-Track Error (e_y)
    double dx = current_pose.x - ref_pt.x;
    double dy = current_pose.y - ref_pt.y;
    double e_y = -std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;

    // 4. Calculate Heading Error (e_theta)
    double e_theta = path_yaw - current_pose.yaw;
    while (e_theta > M_PI)  e_theta -= 2.0 * M_PI;
    while (e_theta < -M_PI) e_theta += 2.0 * M_PI;

    // 5. Stanley Steering Law
    double linear_vel = config_.target_speed;
    double delta = e_theta + std::atan2(config_.k_gain * e_y, linear_vel + config_.k_s);

    // 6. Map steering angle to differential angular velocity (omega)
    double omega = (linear_vel * std::tan(delta)) / config_.wheelbase;
    omega = std::clamp(omega, -config_.max_omega, config_.max_omega);

    cmd.linear = linear_vel;
    cmd.angular = omega;
    return cmd;
}

} // namespace rover::planning