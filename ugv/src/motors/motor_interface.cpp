#include "rover/motors/motor_interface.hpp"
#include <iostream>
#include <sstream>
#include <chrono>

MotorInterface::MotorInterface(RoverState& state, const std::string& com_port, uint32_t baud_rate)
    : state_(state), com_port_(com_port), baud_rate_(baud_rate) {}

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

void MotorInterface::run() {
    // Single consolidated atomic output stream
    std::cout << "[MotorInterface] Attempting connection to ESP32 on " << com_port_ << "...\n" << std::flush;

    if (!serial_.open(com_port_, baud_rate_)) {
        std::cout << "[MotorInterface] WARNING: Could not open " << com_port_ 
                  << ". Running in SERIAL MOCK mode.\n" << std::flush;
    } else {
        std::cout << "[MotorInterface] Connected to ESP32 on " << com_port_ << "!\n" << std::flush;
    }

    while (running_) {
        MotorCommand current_cmd;

        // 1. Fetch current target command from shared state
        {
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            current_cmd = state_.motor_command;
        }

        // 2. Transmit command to ESP32 (Format: "VEL,<linear>,<angular>\n")
        if (serial_.is_open()) {
            std::ostringstream ss;
            ss << "VEL," << current_cmd.linear << "," << current_cmd.angular << "\n";
            serial_.write_string(ss.str());

            // 3. Read incoming encoder odometry from ESP32
            std::string line;
            while (serial_.read_line(line)) {
                parse_encoder_line(line);
            }
        } else {
            // Mock feedback loop when no real physical ESP32 is attached
            std::lock_guard<std::mutex> lock(state_.state_mutex);
            state_.encoders.left_velocity = current_cmd.linear;
            state_.encoders.right_velocity = current_cmd.linear;
            state_.encoders.valid = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz loop
    }

    // Safety shutdown: send zero velocity on exit
    if (serial_.is_open()) {
        serial_.write_string("VEL,0.0,0.0\n");
        serial_.close();
    }
    std::cout << "[MotorInterface] Stopped.\n" << std::flush;
}

void MotorInterface::parse_encoder_line(const std::string& line) {
    // Protocol format: "ENC,<left_dist>,<right_dist>,<left_vel>,<right_vel>"
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
            state_.encoders.left_distance = l_dist;
            state_.encoders.right_distance = r_dist;
            state_.encoders.left_velocity = l_vel;
            state_.encoders.right_velocity = r_vel;
            state_.encoders.valid = true;
        } catch (...) {
            // Drop corrupt frame gracefully
        }
    }
}