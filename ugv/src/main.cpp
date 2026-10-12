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

// Helper to get current unix timestamp for UGV protocol messages
double get_timestamp() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

// Robust helper function to extract numeric values from JSON strings without substring offset bugs
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

int main() {
    std::cout << "===========================================\n";
    std::cout << "        Rover Navigation System Core       \n";
    std::cout << "===========================================\n" << std::flush;

    RoverState state;

    // 1. Initialize Mock Motor Driver, Debug HTTP Server, & UGV UDP Multicast Bus
    MotorInterface motor_driver(state, "COM4", 115200, "../sims/sim_esp.dat");
    motor_driver.start();

    DebugServer debug_server(state, 5000);
    debug_server.start();

    // Multicast group 239.255.42.42 on localhost (127.0.0.1)
    rover::comms::UdpBus udp_bus("239.255.42.42", "127.0.0.1");

    // 2. Initialize Navigation & Pose Estimator
    rover::localization::PoseEstimator pose_estimator;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Capture starting pose from state
    Pose start_pose;
    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.gps.rtk_fixed = false;
        start_pose = state.pose;
    }

    pose_estimator.set_pose(start_pose.x, start_pose.y, start_pose.yaw);

    // 3. Initialize Planner, PurePursuit Follower, and Control States
    std::vector<Pose> global_path;
    
    // PurePursuit controller matching simulation (max_speed=0.8, max_turn=1.0, lookahead=0.6, goal_tol=0.25)
    rover::planning::PurePursuit follower(0.8, 1.0, 0.6, 0.25);

    bool is_teleop_active = false;
    bool has_active_goal = false;

    // Construct Occupancy Grid (50x50 cells at 0.30m resolution = 15m x 15m map frame window)
    const int map_w = 50;
    const int map_h = 50;
    const double res = 0.30;
    std::vector<uint8_t> synthetic_grid(map_w * map_h, 0); // 0 = free

    rover::planning::ThetaStarPlanner planner(map_w, map_h, res);

    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.global_path = global_path;
    }

    std::cout << "\n[Main] Rover online. Waiting for commands from visualiser...\n\n" << std::flush;

    auto last_loop_time = std::chrono::steady_clock::now();
    auto last_log_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_loop_time).count() / 1000000.0;
        last_loop_time = now;

        if (dt > 0.1) dt = 0.05; // Clamp delta time spike on startup

        // --- LISTEN FOR INCOMING BUS MESSAGES (Poses, Goals, LiDAR, Teleop Commands) ---
        auto incoming = udp_bus.poll_messages();
        for (const auto& msg : incoming) {
            
            // A. Check for explicit manual Teleop keypress actions ONLY (prevents cmd_vel loop traps)
            if (msg.find("\"action\":\"teleop\"") != std::string::npos) {
                is_teleop_active = true;
                has_active_goal = false;
                global_path.clear();
                follower.set_path({});
                continue;
            }

            // B. Handle goal cancellation requests from visualiser
            if (msg.find("\"action\":\"cancel\"") != std::string::npos) {
                std::cout << "[Bus] Received goal cancel command from visualiser.\n";
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

            // C. Handle incoming goal commands ("action":"go")
            if (msg.find("\"action\":\"go\"") != std::string::npos || (msg.find("go") != std::string::npos && msg.find("x") != std::string::npos && msg.find("action") != std::string::npos)) {
                double gx = extract_json_val(msg, "x");
                double gy = extract_json_val(msg, "y");

                Pose current_pose;
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    current_pose = state.pose;
                }

                Pose goal_pose{gx, gy, 0.0};
                std::cout << "[Planner] New goal received: (" << gx << ", " << gy << "). Planning path from current pose (" << current_pose.x << ", " << current_pose.y << ")...\n";

                // Plan path using Theta* with occupancy map
                global_path = planner.plan_path(current_pose, goal_pose, synthetic_grid);
                if (global_path.empty()) {
                    std::cerr << "[WARNING] Theta* path generation failed! Falling back to direct line path.\n";
                    global_path = {current_pose, goal_pose};
                }

                // Feed fresh path into PurePursuit follower and enable active goal
                follower.set_path(global_path);
                has_active_goal = true;
                is_teleop_active = false; // Explicitly drop teleop lock when a goal is clicked

                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.global_path = global_path;
                }

                // Broadcast path JSON back to visualiser on port 5008[cite: 1]
                std::ostringstream path_json;
                path_json << "{\"type\":\"path\",\"v\":1,\"ts\": " << get_timestamp() << ",\"points\":[";
                for (size_t i = 0; i < global_path.size(); ++i) {
                    path_json << "[" << global_path[i].x << "," << global_path[i].y << "]";
                    if (i + 1 < global_path.size()) path_json << ",";
                }
                path_json << "]}";
                udp_bus.send(5008, path_json.str());
                std::cout << "[Planner] Path published! Total Waypoints: " << global_path.size() << "\n";
                continue;
            }

            // D. Handle incoming pose telemetry directly as ground truth
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

                // Synchronize pose estimator with true RTK ground truth
                pose_estimator.set_pose(px, py, pyaw);
                update_rover_pos_file("rtk/rover_pos.dat", {px, py, pyaw});
            }

            // E. Handle incoming rtk GNSS fix info for lat/lon display
            if (msg.find("\"type\":\"rtk\"") != std::string::npos) {
                double lat = extract_json_val(msg, "lat");
                double lon = extract_json_val(msg, "lon");
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.gps.latitude = lat;
                    state.gps.longitude = lon;
                }
            }

            // F. Handle incoming RPLidar Scan data over UDP (port 5005)[cite: 1]
            if (msg.find("lidar") != std::string::npos && msg.find("points") != std::string::npos) {
                size_t p_pos = msg.find("\"points\"");
                if (p_pos != std::string::npos) {
                    size_t bracket_pos = msg.find('[', p_pos);
                    if (bracket_pos != std::string::npos && msg.find('[', bracket_pos + 1) != std::string::npos) {
                        std::fill(synthetic_grid.begin(), synthetic_grid.end(), 0); // Clear old scan frame

                        Pose current_pose_local;
                        {
                            std::lock_guard<std::mutex> lock(state.state_mutex);
                            current_pose_local = state.pose;
                        }

                        // Fixed-frame origin anchoring so obstacles don't slide or warp around with the rover
                        double grid_origin_x = current_pose_local.x - 7.5;
                        double grid_origin_y = current_pose_local.y - 7.5;

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
                                        double world_heading = current_pose_local.yaw + body_angle;

                                        double px = current_pose_local.x + dist_m * std::cos(world_heading);
                                        double py = current_pose_local.y + dist_m * std::sin(world_heading);

                                        int gx = static_cast<int>(std::floor((px - grid_origin_x) / res));
                                        int gy = static_cast<int>(std::floor((py - grid_origin_y) / res));

                                        if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h) {
                                            synthetic_grid[gy * map_w + gx] = 1;
                                        }
                                    }
                                } catch (...) {}
                            }
                            curr = pair_end + 1;
                        }
                    }
                }
            }
        }

        // --- 20 Hz CONTROL LOOP STEP ---
        if (dt > 0.0) {
            Pose current_pose;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                current_pose = state.pose;
            }

            // Control Command Arbitration (Teleop > Active Goal > Idle)
            if (is_teleop_active) {
                // Teleop active: leave manual control open
            } else if (has_active_goal) {
                auto [cmd, done] = follower.update(current_pose.x, current_pose.y, current_pose.yaw);

                if (done) {
                    std::cout << "[Planner] Destination reached!\n";
                    has_active_goal = false;
                    is_teleop_active = false;
                    global_path.clear();
                    follower.set_path({});
                    cmd.linear = 0.0;
                    cmd.angular = 0.0;
                }

                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.velocity = cmd;
                    state.motor_command.linear = cmd.linear;
                    state.motor_command.angular = cmd.angular;
                    state.motor_command.enabled = true;
                }

                // Broadcast cmd_vel over UGV protocol on port 5006[cite: 1]
                std::ostringstream cmd_json;
                cmd_json << "{\"type\":\"cmd_vel\",\"v\":1,\"ts\":" << get_timestamp() 
                         << ",\"linear\":" << cmd.linear 
                         << ",\"angular\":" << cmd.angular << "}";
                udp_bus.send(5006, cmd_json.str());
            } else {
                // Idle state: safely disable motor commands without terminating app
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.velocity = {0.0, 0.0};
                state.motor_command.linear = 0.0;
                state.motor_command.angular = 0.0;
                state.motor_command.enabled = false;
            }
        }

        // --- TELEMETRY OUTPUT (Every 1 Second) ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= 1000) {
            Pose p;
            Velocity v;
            double lat = 0, lon = 0;
            bool rtk = false;

            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                p = state.pose;
                v = state.velocity;
                lat = state.gps.latitude;
                lon = state.gps.longitude;
                rtk = state.gps.rtk_fixed;
            }

            std::cout << "[NAV] Pose: (" << std::fixed << std::setprecision(2) << p.x << ", " << p.y << ") m "
                      << "| Yaw: " << std::setprecision(2) << p.yaw << " rad "
                      << "| Cmd (v/w): " << v.linear << " m/s / " << v.angular << " rad/s "
                      << "| Mode: " << (is_teleop_active ? "TELEOP" : (has_active_goal ? "NAVIGATING" : "IDLE")) << " "
                      << "| GPS: " << std::setprecision(5) << lat << "," << lon << " [" << (rtk ? "RTK" : "FLOAT") << "]"
                      << std::endl;

            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 Hz
    }

    motor_driver.stop();
    debug_server.stop();

    return 0;
}