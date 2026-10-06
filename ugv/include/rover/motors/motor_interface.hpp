#pragma once
#include "rover/state/rover_state.hpp"
#include "rover/motors/serial_port.hpp"
#include <atomic>
#include <thread>
#include <string>

class MotorInterface {
public:
    MotorInterface(RoverState& state, const std::string& com_port = "COM4", uint32_t baud_rate = 115200);
    ~MotorInterface();

    void start();
    void stop();

private:
    void run();
    void parse_encoder_line(const std::string& line);

    RoverState& state_;
    std::string com_port_;
    uint32_t baud_rate_;
    SerialPort serial_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};