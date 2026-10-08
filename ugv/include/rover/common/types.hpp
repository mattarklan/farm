#pragma once
#include <cstdint>

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct Velocity {
    double linear = 0.0;
    double angular = 0.0;
};

struct GpsPosition {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    bool valid = false;
    bool rtk_fixed = false;
    int sats{0};
};

struct ImuData {
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double angular_velocity = 0.0;
    double acceleration_x = 0.0;
    double acceleration_y = 0.0;
    bool valid = false;
};

struct EncoderData {
    double left_distance = 0.0;
    double right_distance = 0.0;
    double left_velocity = 0.0;
    double right_velocity = 0.0;
    bool valid = false;
};

struct MotorCommand {
    double linear = 0.0;
    double angular = 0.0;
    bool enabled = false;
};

enum class SystemState {
    INIT,
    IDLE,
    NAVIGATING,
    REPLANNING,
    OBSTACLE_WAIT,
    RETURN_TO_BASE,
    FAULT_STOP,
    TELEOP
};