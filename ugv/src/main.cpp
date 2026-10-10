#include "rover/state/rover_state.hpp"
#include "rover/gps/gps_driver.hpp"
#include "rover/motors/motor_interface.hpp"
#include "rover/debug/debug_server.hpp"
#include "rover/localization/pose_estimator.hpp"
#include "rover/planning/theta_star.hpp"
#include "rover/planning/local_planner.hpp"
#include "rover/comms/udp_bus.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <sstream>

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

int main() {
    std::cout << "===========================================\n";
    std::cout << "       Rover Navigation System Core        \n";
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

    // 3. Define Relative Goal Target (+4.0m East, +4.0m North from start)
    Pose goal_pose;
    goal_pose.x = start_pose.x + 4.0;
    goal_pose.y = start_pose.y + 4.0;
    goal_pose.yaw = 0.0;

    // 4. Construct Occupancy Grid centered around Start Pose (50x50 cells at 0.30m resolution = 15m x 15m)
    const int map_w = 50;
    const int map_h = 50;
    const double res = 0.30;
    std::vector<uint8_t> synthetic_grid(map_w * map_h, 0); // 0 = free

    // Place synthetic obstacle block in local map frame
    for (int y = 7; y <= 12; ++y) {
        for (int x = 7; x <= 12; ++x) {
            synthetic_grid[y * map_w + x] = 1; // 1 = occupied
        }
    }

    rover::planning::ThetaStarPlanner planner(map_w, map_h, res);
    rover::planning::LocalPlanner controller;

    // 5. Generate Dynamic Relative Path
    std::cout << "[Planner] Planning relative path from (" 
              << start_pose.x << ", " << start_pose.y << ") to (" 
              << goal_pose.x << ", " << goal_pose.y << ")...\n" << std::flush;

    std::vector<Pose> global_path = planner.plan_path(start_pose, goal_pose, synthetic_grid);

    if (global_path.empty()) {
        std::cerr << "[WARNING] Theta* path generation failed! Falling back to direct line path.\n";
        global_path = {start_pose, goal_pose};
    }

    std::cout << "[Planner] Path found! Total Waypoints: " << global_path.size() << "\n";
    for (size_t i = 0; i < global_path.size(); ++i) {
        std::cout << "  - Waypoint [" << i << "]: ENU (" 
                  << std::fixed << std::setprecision(2) << global_path[i].x << ", " 
                  << global_path[i].y << ") Yaw: " << global_path[i].yaw << " rad\n";
    }

    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.global_path = global_path;
    }

    // Broadcast generated path over UGV protocol on port 5008 so the visualiser draws it
    {
        std::ostringstream path_json;
        path_json << "{\"type\":\"path\",\"v\":1,\"ts\": " << get_timestamp() << ",\"points\":[";
        for (size_t i = 0; i < global_path.size(); ++i) {
            path_json << "[" << global_path[i].x << "," << global_path[i].y << "]";
            if (i + 1 < global_path.size()) path_json << ",";
        }
        path_json << "]}";
        udp_bus.send(5008, path_json.str());
    }

    std::cout << "\n[Main] Closed-loop relative motion active. Logging every 1 sec...\n\n" << std::flush;

    auto last_loop_time = std::chrono::steady_clock::now();
    auto last_log_time = std::chrono::steady_clock::now();
    size_t active_waypoint_idx = 1;

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_loop_time).count() / 1000000.0;
        last_loop_time = now;

        if (dt > 0.1) dt = 0.05; // Clamp delta time spike on startup

        // --- 20 Hz CONTROL LOOP STEP ---
        if (dt > 0.0) {
            // A. Integrate Kinematic Pose (Dead reckoning)
            pose_estimator.update(state, dt);

            Pose current_pose;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                current_pose = state.pose;
            }

            // B. Write live simulated ENU pose to rtk/rover_pos.dat in exact JSON format
            update_rover_pos_file("../rtk/rover_pos.dat", current_pose);

            // C. Compute Command Velocity via Stanley Controller
            Velocity cmd = controller.compute_command(current_pose, global_path, active_waypoint_idx);

            // D. Apply Velocity Commands to State
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.velocity = cmd;
                state.motor_command.linear = cmd.linear;
                state.motor_command.angular = cmd.angular;
                state.motor_command.enabled = true;
            }

            // E. Broadcast cmd_vel over UGV protocol on port 5006 for the visualiser
            {
                std::ostringstream cmd_json;
                cmd_json << "{\"type\":\"cmd_vel\",\"v\":1,\"ts\":" << get_timestamp() 
                         << ",\"linear\":" << cmd.linear 
                         << ",\"angular\":" << cmd.angular << "}";
                udp_bus.send(5006, cmd_json.str());
            }
        }

        // --- LISTEN FOR INCOMING BUS MESSAGES (e.g., goal cancels from visualiser) ---
        auto incoming = udp_bus.poll_messages();
        for (const auto& msg : incoming) {
            if (msg.find("\"type\":\"goal\"") != std::string::npos && msg.find("\"action\":\"cancel\"") != std::string::npos) {
                std::cout << "[Bus] Received goal cancel command from visualiser.\n";
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
                      << "| Target WP: [" << active_waypoint_idx << "/" << global_path.size() - 1 << "] "
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