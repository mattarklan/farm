#include "rover/state/rover_state.hpp"
#include "rover/gps/gps_driver.hpp"
#include "rover/motors/motor_interface.hpp"
#include "rover/debug/debug_server.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

int main() {
    std::cout << "===========================================\n";
    std::cout << "       Rover Navigation System Core        \n";
    std::cout << "===========================================\n" << std::flush;

    RoverState state;

    GpsDriver gps_driver(state, "../rtk/rover_pos.dat");
    gps_driver.start();

    MotorInterface motor_driver(state, "COM4", 115200, "../sims/sim_esp.dat");
    motor_driver.start();

    DebugServer debug_server(state, 5000);
    debug_server.start();

    std::cout << "[Main] System initialized. Outputting telemetry every 5s...\n\n" << std::flush;

    auto last_log_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= 5000) {
            
            // Local state snapshot
            double lat = 0, lon = 0;
            bool rtk = false;
            double l_vel = 0, r_vel = 0, yaw = 0;
            bool enc_ok = false, imu_ok = false;

            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                lat = state.gps.latitude;
                lon = state.gps.longitude;
                rtk = state.gps.rtk_fixed;

                l_vel = state.encoders.left_velocity;
                r_vel = state.encoders.right_velocity;
                enc_ok = state.encoders.valid;

                yaw = state.imu.yaw;
                imu_ok = state.imu.valid;
            }

            // Single line print with \r (carriage return) and \033[K (clear line to end)
            std::cout << "\r[TELEMETRY] "
                      << "GPS: " << std::fixed << std::setprecision(5) << lat << "," << lon 
                      << " [" << (rtk ? "RTK" : "FLOAT") << "] | "
                      << "ENC(L/R): " << std::setprecision(2) << l_vel << "/" << r_vel 
                      << " [" << (enc_ok ? "OK" : "NO_DATA") << "] | "
                      << "IMU Yaw: " << yaw 
                      << " [" << (imu_ok ? "OK" : "NO_DATA") << "]\033[K" << std::flush;

            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    gps_driver.stop();
    motor_driver.stop();
    debug_server.stop();

    return 0;
}