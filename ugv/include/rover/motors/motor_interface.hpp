#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP
#include "serial_port.hpp"
#include "rover/state/rover_state.hpp"
#include <string>
#include <thread>
#include <mutex>

// Wrapper/Mock interface for serial port

class MotorInterface {
public:
    MotorInterface(RoverState& state, 
                   const std::string& com_port = "COM4", 
                   uint32_t baud_rate = 115200, 
                   const std::string& sim_file = "../sims/sim_esp.dat");
    ~MotorInterface();

    void start();
    void stop();

private:
    void run();
    void parse_telemetry_line(const std::string& line);
    bool read_sim_telemetry();

    RoverState& state_;
    std::string com_port_;
    uint32_t baud_rate_;
    std::string sim_file_;
    
    SerialPort serial_;
    bool running_{false};
    bool using_sim_file_{false};
    std::thread thread_;
};

#endif // MOTOR_INTERFACE_HPP