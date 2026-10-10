#pragma once

#include "rover/common/types.hpp"
#include <mutex>
#include <vector>

struct RoverState {
    GpsPosition gps;
    ImuData imu;
    EncoderData encoders;
    Pose pose;
    Velocity velocity;
    MotorCommand motor_command;
    SystemState state = SystemState::INIT;
    bool emergency_stop = false;

    // Added to store active Theta* global navigation waypoints
    std::vector<Pose> global_path;

    std::mutex state_mutex;
};