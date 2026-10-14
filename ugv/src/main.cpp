#include "rover/planning/lcostmap.hpp"
#include "rover/state/rover_state.hpp"
#include "rover/gps/gps_driver.hpp"
#include "rover/motors/motor_interface.hpp"
#include "rover/debug/debug_server.hpp"
#include "rover/localization/pose_estimator.hpp"
#include "rover/planning/theta_star.hpp"
#include "rover/planning/follower.hpp"
#include "rover/comms/udp_bus.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cstdint>

// Helper to update rtk/rover_pos.dat with live JSON pose telemetry
void update_rover_pos_file(const std::string& filepath, const Pose& pose) {
    std::ofstream outfile(filepath, std::ios::trunc);
    if (outfile.is_open()) {
        auto now = std::chrono::system_clock::now();
        double ts = std::chrono::duration<double>(now.time_since_epoch()).count();

        outfile << std::fixed << std::setprecision(9)
                << "{\"lat\": -31.950500067"
                << ", \"lon\": 115.860489505"
                << ", \"fix\": \"Fixed RTK\""
                << ", \"sats\": 23"
                << ", \"hdop\": 0.67"
                << ", \"corr_age\": 0.8"
                << ", \"ts\": " << std::setprecision(7) << ts
                << ", \"enu_x\": " << std::setprecision(2) << pose.x
                << ", \"enu_y\": " << pose.y
                << ", \"yaw\": " << std::setprecision(4) << pose.yaw
                << "}\n";
    }
}

double get_timestamp() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

double extract_json_val(const std::string& json_str, const std::string& key) {
    std::string search_key = "\"" + key + "\"";
    size_t key_pos = json_str.find(search_key);
    if (key_pos == std::string::npos) return 0.0;

    size_t colon_pos = json_str.find(':', key_pos);
    if (colon_pos == std::string::npos) return 0.0;

    size_t val_start = colon_pos + 1;
    while (val_start < json_str.size() && (json_str[val_start] == ' ' || json_str[val_start] == '\t' || json_str[val_start] == '\"')) {
        val_start++;
    }

    if (val_start >= json_str.size()) return 0.0;

    try {
        return std::stod(json_str.substr(val_start));
    } catch (...) {
        return 0.0;
    }
}

// Robust Occupancy Map with dynamic re-centering support
class OccupancyMap {
public:
    OccupancyMap(int w, int h, double res) : w_(w), h_(h), res_(res), lo_(w * h, 0) {}

    bool initialised() const { return init_; }

    void init(double cx, double cy) {
        ox_ = cx - (w_ * res_) / 2.0;
        oy_ = cy - (h_ * res_) / 2.0;
        init_ = true;
    }

    // Shift map origin dynamically if rover approaches boundaries, keeping rover centered
    void checkAndRecentering(double rx, double ry) {
        if (!init_) {
            init(rx, ry);
            return;
        }
        double margin = 5.0; // meters from edge
        double cur_w = w_ * res_;
        double cur_h = h_ * res_;

        if (rx < ox_ + margin || rx > ox_ + cur_w - margin ||
            ry < oy_ + margin || ry > oy_ + cur_h - margin) {
            std::cout << "[OccupancyMap] Re-centering map around rover at (" << rx << ", " << ry << ")\n";
            init(rx, ry); // Re-anchor centered on current position
            std::fill(lo_.begin(), lo_.end(), 0); // Reset log-odds cleanly on re-center
        }
    }

    double originX() const { return ox_; }
    double originY() const { return oy_; }

    bool toCell(double wx, double wy, int& cx, int& cy) const {
        cx = static_cast<int>(std::floor((wx - ox_) / res_));
        cy = static_cast<int>(std::floor((wy - oy_) / res_));
        return inBounds(cx, cy);
    }

    void integrate(double rx, double ry, double ryaw, const std::vector<std::pair<double, double>>& scans) {
        checkAndRecentering(rx, ry);
        int sx, sy;
        if (!toCell(rx, ry, sx, sy)) return;

        for (const auto& s : scans) {
            double hx = rx + s.first * std::cos(s.second + ryaw);
            double hy = ry + s.first * std::sin(s.second + ryaw);
            int ex = static_cast<int>(std::floor((hx - ox_) / res_));
            int ey = static_cast<int>(std::floor((hy - oy_) / res_));

            traceFree(sx, sy, ex, ey);
            if (inBounds(ex, ey)) bump(ex, ey, +5);
        }
    }

    std::vector<uint8_t> buildPlanningGrid(double rx, double ry, int inflate_cells, int clear_cells) const {
        std::vector<uint8_t> g(w_ * h_, 0);
        const int r2 = inflate_cells * inflate_cells;

        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                if (lo_[y * w_ + x] < kOccThresh) continue;
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        if (dx * dx + dy * dy > r2) continue;
                        int nx = x + dx, ny = y + dy;
                        if (!inBounds(nx, ny)) continue;
                        uint8_t& c = g[ny * w_ + nx];
                        if (c == 0) c = 1;
                    }
                }
            }
        }
        for (int i = 0; i < w_ * h_; ++i) {
            if (lo_[i] >= kOccThresh) g[i] = 2;
        }

        int rcx, rcy;
        if (toCell(rx, ry, rcx, rcy)) {
            const int c2 = clear_cells * clear_cells;
            for (int dy = -clear_cells; dy <= clear_cells; ++dy) {
                for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
                    if (dx * dx + dy * dy > c2) continue;
                    int nx = rcx + dx, ny = rcy + dy;
                    if (inBounds(nx, ny) && g[ny * w_ + nx] == 1) g[ny * w_ + nx] = 0;
                }
            }
        }
        return g;
    }

private:
    static constexpr int kOccThresh = 3;

    bool inBounds(int x, int y) const { return x >= 0 && x < w_ && y >= 0 && y < h_; }

    void bump(int x, int y, int d) {
        int8_t& v = lo_[y * w_ + x];
        int nv = static_cast<int>(v) + d;
        v = static_cast<int8_t>(std::clamp(nv, -10, 30));
    }

    void traceFree(int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (!(x == x1 && y == y1)) {
            if (inBounds(x, y)) bump(x, y, -1);
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }
    }

    int w_, h_;
    double res_;
    double ox_{0.0}, oy_{0.0};
    bool init_{false};
    std::vector<int8_t> lo_;
};

int main() {
    std::cout << "===========================================\n";
    std::cout << "    Rover Navigation System (Robust Core)  \n";
    std::cout << "===========================================\n" << std::flush;

    RoverState state;

    MotorInterface motor_driver(state, "COM4", 115200, "../sims/sim_esp.dat");
    motor_driver.start();

    DebugServer debug_server(state, 5000);
    debug_server.start();

    rover::comms::UdpBus udp_bus("239.255.42.42", "127.0.0.1");
    rover::localization::PoseEstimator pose_estimator;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Pose start_pose;
    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.gps.rtk_fixed = false;
        start_pose = state.pose;
    }

    pose_estimator.set_pose(start_pose.x, start_pose.y, start_pose.yaw);

    std::vector<Pose> global_path;
    rover::planning::PurePursuit follower(0.8, 1.0, 0.6, 0.25);

    bool is_teleop_active = false;
    bool has_active_goal = false;

    // Generous map size (600x600 cells at 0.10m = 60m x 60m)
    const int map_w = 600;
    const int map_h = 600;
    const double res = 0.10;
    const double kInflateM = 0.45; 
    const double kClearM   = 0.35; 
    OccupancyMap world_map(map_w, map_h, res);

    const double local_res = 0.05;
    LocalCostmap local_costmap(200, 200, local_res);

    rover::planning::ThetaStarPlanner planner(map_w, map_h, res);

    Pose active_goal{0.0, 0.0, 0.0};
    auto last_replan_time = std::chrono::steady_clock::now();
    double forward_clearance = 99.0;

    auto plan_and_publish = [&](const Pose& from, const Pose& goal, bool log) -> bool {
        world_map.checkAndRecentering(from.x, from.y);
        planner.set_origin(world_map.originX(), world_map.originY());

        auto plan_grid = world_map.buildPlanningGrid(
            from.x, from.y,
            static_cast<int>(std::round(kInflateM / res)),
            static_cast<int>(std::round(kClearM / res)));

        auto path = planner.plan_path(from, goal, plan_grid);
        if (path.empty()) {
            std::cerr << "[Planner ERROR] No path available from current position (" 
                      << from.x << ", " << from.y << ") to goal (" << goal.x << ", " << goal.y << ").\n";
            return false;
        }

        global_path = path;
        follower.set_path(global_path);
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.global_path = global_path;
        }

        std::ostringstream path_json;
        path_json << "{\"type\":\"path\",\"v\":1,\"ts\": " << get_timestamp() << ",\"points\":[";
        for (size_t i = 0; i < global_path.size(); ++i) {
            path_json << "[" << global_path[i].x << "," << global_path[i].y << "]";
            if (i + 1 < global_path.size()) path_json << ",";
        }
        path_json << "]}";
        udp_bus.send(5008, path_json.str());
        if (log) std::cout << "[Planner] Path published successfully! Waypoints: " << global_path.size() << "\n";
        return true;
    };

    std::cout << "\n[Main] Rover online and waiting for commands...\n\n" << std::flush;

    auto last_loop_time = std::chrono::steady_clock::now();
    auto last_log_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_loop_time).count() / 1000000.0;
        last_loop_time = now;
        if (dt > 0.1) dt = 0.05;

        // --- POLL INCOMING UDP MESSAGES ---
        auto incoming = udp_bus.poll_messages();
        for (const auto& msg : incoming) {
            
            // 1. Teleop command explicitly takes precedence and interrupts autonomous navigation
            if (msg.find("\"action\":\"teleop\"") != std::string::npos || 
                msg.find("\"action\":\"key\"") != std::string::npos) {
                is_teleop_active = true;
                has_active_goal = false;
                global_path.clear();
                follower.set_path({});
                continue;
            }

            // 2. Goal cancellation request
            if (msg.find("\"action\":\"cancel\"") != std::string::npos) {
                std::cout << "[Bus] Goal cancelled by user.\n";
                global_path.clear();
                has_active_goal = false;
                is_teleop_active = false;
                follower.set_path({});
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.velocity = {0.0, 0.0};
                    state.motor_command.linear = 0.0;
                    state.motor_command.angular = 0.0;
                    state.motor_command.enabled = false;
                }
                continue;
            }

            // 3. New Goal Request ("action":"go")
            if (msg.find("\"action\":\"go\"") != std::string::npos || (msg.find("go") != std::string::npos && msg.find("x") != std::string::npos)) {
                double gx = extract_json_val(msg, "x");
                double gy = extract_json_val(msg, "y");

                Pose current_pose;
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    current_pose = state.pose;
                }

                active_goal = Pose{gx, gy, 0.0};
                std::cout << "[Planner] New goal received: (" << gx << ", " << gy << "). Planning from current pose (" << current_pose.x << ", " << current_pose.y << ")...\n";

                is_teleop_active = false; // Teleop overridden by new goal click

                if (!plan_and_publish(current_pose, active_goal, true)) {
                    has_active_goal = false;
                    global_path.clear();
                    follower.set_path({});
                    continue;
                }

                has_active_goal = true;
                last_replan_time = now;
                continue;
            }

            // 4. Pose Telemetry
            if (msg.find("\"x\"") != std::string::npos && msg.find("\"yaw\"") != std::string::npos && msg.find("\"action\"") == std::string::npos) {
                double px = extract_json_val(msg, "x");
                double py = extract_json_val(msg, "y");
                double pyaw = extract_json_val(msg, "yaw");

                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.pose.x = px;
                    state.pose.y = py;
                    state.pose.yaw = pyaw;
                    state.gps.rtk_fixed = (msg.find("Fixed RTK") != std::string::npos);
                }

                pose_estimator.set_pose(px, py, pyaw);
                update_rover_pos_file("rtk/rover_pos.dat", {px, py, pyaw});
            }

            // 5. LiDAR Scan Data processing (Port 5005)
            if (msg.find("lidar") != std::string::npos && msg.find("points") != std::string::npos) {
                size_t p_pos = msg.find("\"points\"");
                if (p_pos != std::string::npos) {
                    size_t bracket_pos = msg.find('[', p_pos);
                    if (bracket_pos != std::string::npos) {
                        Pose current_pose_local;
                        {
                            std::lock_guard<std::mutex> lock(state.state_mutex);
                            current_pose_local = state.pose;
                        }

                        if (!world_map.initialised()) world_map.init(current_pose_local.x, current_pose_local.y);

                        std::vector<std::pair<double, double>> local_lidar_scans;
                        size_t curr = bracket_pos;
                        while (true) {
                            size_t pair_start = msg.find('[', curr);
                            if (pair_start == std::string::npos) break;
                            size_t pair_end = msg.find(']', pair_start);
                            if (pair_end == std::string::npos) break;

                            std::string pair_str = msg.substr(pair_start + 1, pair_end - pair_start - 1);
                            size_t comma = pair_str.find(',');
                            if (comma != std::string::npos) {
                                try {
                                    double angle_deg = std::stod(pair_str.substr(0, comma));
                                    double dist_mm = std::stod(pair_str.substr(comma + 1));
                                    double dist_m = dist_mm / 1000.0;

                                    if (dist_m > 0.1 && dist_m <= 12.0) {
                                        double body_angle = -angle_deg * M_PI / 180.0;
                                        local_lidar_scans.push_back({dist_m, body_angle});
                                    }
                                } catch (...) {}
                            }
                            curr = pair_end + 1;
                        }

                        // Integrate LiDAR into occupancy grid for obstacle avoidance
                        world_map.integrate(current_pose_local.x, current_pose_local.y,
                                            current_pose_local.yaw, local_lidar_scans);

                        local_costmap.update(local_lidar_scans, current_pose_local.x, current_pose_local.y, current_pose_local.yaw);

                        // Check forward clearance directly from LiDAR
                        const auto& lgrid = local_costmap.getGrid();
                        const int cw = local_costmap.getWidth();
                        const int ch = local_costmap.getHeight();
                        const double hx = std::cos(current_pose_local.yaw);
                        const double hy = std::sin(current_pose_local.yaw);

                        forward_clearance = 99.0;
                        for (double d = 0.15; d <= 1.5 && forward_clearance > 98.0; d += local_res) {
                            for (double lat : {-0.2, 0.0, 0.2}) {
                                double ddx = d * hx - lat * hy;
                                double ddy = d * hy + lat * hx;
                                int cx = static_cast<int>(std::floor(ddx / local_res)) + cw / 2;
                                int cy = static_cast<int>(std::floor(ddy / local_res)) + ch / 2;
                                if (cx < 0 || cx >= cw || cy < 0 || cy >= ch) continue;
                                if (lgrid[cy * cw + cx].cost >= 100) {
                                    forward_clearance = d;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }

        // --- 20 Hz CONTROL LOOP ---
        if (dt > 0.0) {
            Pose current_pose;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                current_pose = state.pose;
            }

            if (is_teleop_active) {
                // Manual teleop active: controller loop lets manual bus commands pass through
            } else if (has_active_goal) {
                // Replan periodically to account for newly discovered LiDAR obstacles
                if (std::chrono::duration<double>(now - last_replan_time).count() >= 0.5) {
                    last_replan_time = now;
                    if (!plan_and_publish(current_pose, active_goal, false)) {
                        std::cerr << "[Planner] Replan temporarily blocked. Retrying next cycle...\n";
                    }
                }

                auto [cmd, done] = follower.update(current_pose.x, current_pose.y, current_pose.yaw);

                if (done) {
                    std::cout << "[Planner] Destination successfully reached!\n";
                    has_active_goal = false;
                    is_teleop_active = false;
                    global_path.clear();
                    follower.set_path({});
                    cmd.linear = 0.0;
                    cmd.angular = 0.0;
                }

                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    if (!done && cmd.linear > 0.0) {
                        double scale = 1.0;
                        if (forward_clearance < 0.35)      scale = 0.0;
                        else if (forward_clearance < 1.0)  scale = (forward_clearance - 0.35) / 0.65;
                        cmd.linear *= scale;
                    }
                    state.velocity = cmd;
                    state.motor_command.linear = cmd.linear;
                    state.motor_command.angular = cmd.angular;
                    state.motor_command.enabled = true;
                }

                std::ostringstream cmd_json;
                cmd_json << "{\"type\":\"cmd_vel\",\"v\":1,\"ts\":" << get_timestamp() 
                         << ",\"linear\":" << cmd.linear 
                         << ",\"angular\":" << cmd.angular << "}";
                udp_bus.send(5006, cmd_json.str());
            } else {
                // Idle state: safely disengage motors without shutting down app
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.velocity = {0.0, 0.0};
                state.motor_command.linear = 0.0;
                state.motor_command.angular = 0.0;
                state.motor_command.enabled = false;
            }
        }

        // --- TELEMETRY CONSOLE OUTPUT (Every 1 Second) ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= 1000) {
            Pose p;
            Velocity v;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                p = state.pose;
                v = state.velocity;
            }

            std::cout << "[NAV] Pose: (" << std::fixed << std::setprecision(2) << p.x << ", " << p.y << ") m "
                      << "| Yaw: " << std::setprecision(2) << p.yaw << " rad "
                      << "| Mode: " << (is_teleop_active ? "TELEOP" : (has_active_goal ? "NAVIGATING" : "IDLE"))
                      << std::endl;

            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    motor_driver.stop();
    debug_server.stop();
    return 0;
}