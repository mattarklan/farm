#include "rover/motors/motor_interface.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

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
    while (std::getline(file, line)) {
        parse_telemetry_line(line);
    }
    return true;
}

void MotorInterface::parse_telemetry_line(const std::string& line) {
    // Protocol 1: "ENC,<left_dist>,<right_dist>,<left_vel>,<right_vel>"
    if (line.rfind("ENC,", 0) == 0) {
        std::stringstream ss(line.substr(4));
        std::string token;
        double l_dist = 0, r_dist = 0, l_vel = 0, r_vel = 0;

        try {
            if (std::getline(ss, token, ',')) l_dist = std::stod(token);
            if (std::getline(ss, token, ',')) r_dist = std::stod(token);
            if (std::getline(ss, token, ',')) l_vel = std::stod(token);
            if (std::getline(ss, token, ',')) r_vel = std::stod(token);

            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.encoders.left_distance  = l_dist;
            state_.encoders.right_distance = r_dist;
            state_.encoders.left_velocity  = l_vel;
            state_.encoders.right_velocity = r_vel;
            state_.encoders.valid          = true;
        } catch (...) {}
    }
    // Protocol 2: "IMU,<yaw>,<pitch>,<roll>"
else if (line.rfind("IMU,", 0) == 0) {
        std::stringstream ss(line.substr(4));
        std::string token;
        double yaw = 0, pitch = 0, roll = 0;

        try {
            if (std::getline(ss, token, ',')) yaw   = std::stod(token);
            if (std::getline(ss, token, ',')) pitch = std::stod(token);
            if (std::getline(ss, token, ',')) roll  = std::stod(token);

            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.imu.yaw   = yaw;     // If your struct uses 'heading', change this to state_.imu.heading = yaw;
            state_.imu.pitch = pitch;
            state_.imu.roll  = roll;
            state_.imu.valid = true;

            state_.pose.yaw = yaw;
        } catch (...) {}
    }
}

void MotorInterface::run() {
    std::cout << "[MotorInterface] Attempting connection to " << com_port_ << "...\n" << std::flush;

    if (serial_.open(com_port_, baud_rate_)) {
        using_sim_file_ = false;
        std::cout << "[MotorInterface] Connected to hardware on " << com_port_ << ".\n" << std::flush;
    } else {
        using_sim_file_ = true;
        std::cout << "[MotorInterface] " << com_port_ << " NOT detected. Falling back to simulation mode (" << sim_file_ << ").\n" << std::flush;
    }

    while (running_) {
        MotorCommand current_cmd;

        // Fetch command from shared state
        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            current_cmd = state_.motor_command;
        }

        if (!using_sim_file_ && serial_.is_open()) {
            // --- COM4 HARDWARE MODE ---
            // 1. Send VEL command
            std::ostringstream ss;
            ss << "VEL," << current_cmd.linear << "," << current_cmd.angular << "\n";
            serial_.write_string(ss.str());

            // 2. Read incoming serial line for ENC/IMU feedback
            std::string rx_line;
            while (serial_.read_line(rx_line)) {
                parse_telemetry_line(rx_line);
            }
        } else {
            // --- SIMULATION MODE ---
            if (!read_sim_telemetry()) {
                // Fallback: Mirror commands directly to state if sim file is also missing
                std::lock_guard<std::mutex> lock(state_.state_mutex);
                state_.encoders.left_velocity  = current_cmd.linear;
                state_.encoders.right_velocity = current_cmd.linear;
                state_.encoders.valid          = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz loop
    }

    if (serial_.is_open()) {
        serial_.write_string("VEL,0.0,0.0\n");
        serial_.close();
    }
    std::cout << "[MotorInterface] Stopped.\n" << std::flush;
}