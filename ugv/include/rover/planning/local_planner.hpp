#pragma once

#include "rover/common/types.hpp"
#include <vector>

namespace rover::planning {

struct StanleyConfig {
    double k_gain{0.5};      // Cross-track correction gain
    double k_s{0.1};         // Softening constant for low-speed stability
    double max_omega{1.0};   // Max turning speed (rad/s)
    double wheelbase{0.5};   // Wheelbase / rover length (m)
    double target_speed{0.8}; // Cruise velocity (m/s)
    double goal_tolerance{0.3}; // Target arrival distance (m)
};

class LocalPlanner {
public:
    explicit LocalPlanner(const StanleyConfig& config = StanleyConfig());

    // Computes linear and angular velocity commands to follow global path
    Velocity compute_command(const Pose& current_pose, 
                            const std::vector<Pose>& global_path, 
                            size_t& target_waypoint_idx);

private:
    StanleyConfig config_;
};

} // namespace rover::planning