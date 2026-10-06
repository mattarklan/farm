#pragma once
#include "rover/common/types.hpp"
#include <mutex>

struct RoverState {
    GpsPosition gps;
    ImuData imu;
    EncoderData encoders;
    Pose pose;
    Velocity velocity;
    MotorCommand motor_command;
    SystemState state = SystemState::INIT;
    bool emergency_stop = false;

    std::mutex state_mutex;
};