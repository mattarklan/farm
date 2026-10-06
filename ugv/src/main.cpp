#include "rover/state/rover_state.hpp"
#include "rover/debug/debug_server.hpp"
#include "rover/motors/motor_interface.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\n[Main] Shutdown signal received.\n";
        g_shutdown_requested = true;
    }
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "===========================================\n";
    std::cout << "       Rover Navigation System Core        \n";
    std::cout << "===========================================\n";

    RoverState shared_state;

    // Initialize state
    {
        std::lock_guard<std::mutex> lock(shared_state.state_mutex);
        shared_state.state = SystemState::INIT;
        shared_state.pose = { 10.5, 20.2, 1.57 };
        shared_state.gps.rtk_fixed = true;
        
        // Set an initial command velocity test (0.5 m/s forward)
        shared_state.motor_command.linear = 0.5;
        shared_state.motor_command.angular = 0.0;
        shared_state.motor_command.enabled = true;
    }

    // Start Debug TCP Server (Phase 2)
    DebugServer debug_server(shared_state, 5000);
    debug_server.start();

    // Start Motor Interface Thread (Phase 3)
    // Adjust "COM4" to match your ESP32's COM port if connected
    MotorInterface motor_interface(shared_state, "COM4", 115200);
    motor_interface.start();

    // Transition to IDLE state
    {
        std::lock_guard<std::mutex> lock(shared_state.state_mutex);
        shared_state.state = SystemState::IDLE;
        std::cout << "[Main] System initialized. State transition -> IDLE\n";
    }

    std::cout << "[Main] Running... Press Ctrl+C to stop.\n";

    while (!g_shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "[Main] Stopping subsystems...\n";
    motor_interface.stop();
    debug_server.stop();
    std::cout << "[Main] System shutdown completed safely.\n";

    return 0;
}