#include "rover/motors/motor_interface.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cmath>

static double extract_json_double(const std::string& str, const std::string& key) {
    size_t pos = str.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0.0;
    
    pos = str.find(':', pos);
    if (pos == std::string::npos) return 0.0;
    
    size_t start = str.find_first_of("-0123456789.", pos);
    if (start == std::string::npos) return 0.0;
    
    try {
        return std::stod(str.substr(start));
    } catch (...) {
        return 0.0;
    }
}

MotorInterface::MotorInterface(RoverState& state, const std::string& com_port, 
                               uint32_t baud_rate, const std::string& sim_file)
    : state_(state), com_port_(com_port), baud_rate_(baud_rate), sim_file_(sim_file) {}

MotorInterface::~MotorInterface() {
    stop();
}

void MotorInterface::start() {
    running_ = true;
    thread_ = std::thread(&MotorInterface::run, this);
}

void MotorInterface::stop() {
    if (running_) {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }
}

bool MotorInterface::read_sim_telemetry() {
    std::ifstream file(sim_file_);
    if (!file.is_open()) return false;

    std::string line;
    bool parsed_any = false;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            parse_telemetry_line(line);
            parsed_any = true;
        }
    }
    return parsed_any;
}

void MotorInterface::parse_telemetry_line(const std::string& raw_line) {
    std::string line = raw_line;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) {
        line.pop_back();
    }
    if (line.empty()) return;

    try {
        // --- FORMAT A: JSON Telemetry (sim_esp.dat) ---
        if (line.front() == '{' && line.back() == '}') {
            double l_vel   = extract_json_double(line, "left_vel");
            double r_vel   = extract_json_double(line, "right_vel");
            double l_ticks = extract_json_double(line, "left_ticks");
            double r_ticks = extract_json_double(line, "right_ticks");

            double yaw = 0.0;
            size_t quat_pos = line.find("\"quat\":");
            if (quat_pos != std::string::npos) {
                size_t arr_start = line.find('[', quat_pos);
                size_t arr_end   = line.find(']', arr_start);
                if (arr_start != std::string::npos && arr_end != std::string::npos) {
                    std::string q_str = line.substr(arr_start + 1, arr_end - arr_start - 1);
                    std::stringstream ss(q_str);
                    std::string token;
                    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

                    if (std::getline(ss, token, ',')) qw = std::stod(token);
                    if (std::getline(ss, token, ',')) qx = std::stod(token);
                    if (std::getline(ss, token, ',')) qy = std::stod(token);
                    if (std::getline(ss, token, ',')) qz = std::stod(token);

                    yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
                }
            }

            {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.encoders.left_velocity  = l_vel;
                state_.encoders.right_velocity = r_vel;
                state_.encoders.left_distance  = l_ticks;
                state_.encoders.right_distance = r_ticks;
                state_.encoders.valid          = true;

                state_.imu.yaw   = yaw;
                state_.imu.valid = true;
                state_.pose.yaw  = yaw;
            }
            return;
        }

        // --- FORMAT B: Standard CSV Telemetry ---
        if (line.rfind("ENC,", 0) == 0) {
            std::stringstream ss(line.substr(4));
            std::string token;
            double l_dist = 0, r_dist = 0, l_vel = 0, r_vel = 0;

            if (std::getline(ss, token, ',')) l_dist = std::stod(token);
            if (std::getline(ss, token, ',')) r_dist = std::stod(token);
            if (std::getline(ss, token, ',')) l_vel  = std::stod(token);
            if (std::getline(ss, token, ',')) r_vel  = std::stod(token);

            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.encoders.left_distance  = l_dist;
            state_.encoders.right_distance = r_dist;
            state_.encoders.left_velocity  = l_vel;
            state_.encoders.right_velocity = r_vel;
            state_.encoders.valid          = true;
        } 
        else if (line.rfind("IMU,", 0) == 0) {
            std::stringstream ss(line.substr(4));
            std::string token;
            double yaw = 0, pitch = 0, roll = 0;

            if (std::getline(ss, token, ',')) yaw   = std::stod(token);
            if (std::getline(ss, token, ',')) pitch = std::stod(token);
            if (std::getline(ss, token, ',')) roll  = std::stod(token);

            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.imu.yaw   = yaw;
            state_.imu.pitch = pitch;
            state_.imu.roll  = roll;
            state_.imu.valid = true;
            state_.pose.yaw  = yaw;
        }
    } catch (...) {
        // Suppress parsing exceptions
    }
}

void MotorInterface::run() {
    std::cout << "[MotorInterface] Thread running...\n" << std::flush;

    if (serial_.open(com_port_, baud_rate_)) {
        using_sim_file_ = false;
        std::cout << "[MotorInterface] Connected on " << com_port_ << ".\n" << std::flush;
    } else {
        using_sim_file_ = true;
        std::cout << "[MotorInterface] Fallback mode: Reading " << sim_file_ << "\n" << std::flush;
    }

    while (running_) {
        MotorCommand current_cmd;

        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            current_cmd = state_.motor_command;
        }

        if (!using_sim_file_ && serial_.is_open()) {
            std::ostringstream ss;
            ss << "VEL," << current_cmd.linear << "," << current_cmd.angular << "\n";
            serial_.write_string(ss.str());

            std::string rx_line;
            while (serial_.read_line(rx_line)) {
                parse_telemetry_line(rx_line);
            }
        } else {
            if (!read_sim_telemetry()) {
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.encoders.left_velocity  = current_cmd.linear;
                state_.encoders.right_velocity = current_cmd.linear;
                state_.encoders.valid          = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (serial_.is_open()) {
        serial_.write_string("VEL,0.0,0.0\n");
        serial_.close();
    }
}