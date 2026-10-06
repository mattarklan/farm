#pragma once
#include "rover/state/rover_state.hpp"
#include <atomic>
#include <thread>

class DebugServer {
public:
    DebugServer(RoverState& state, int port = 5000);
    ~DebugServer();

    void start();
    void stop();

private:
    void run();
    void cleanup_winsock();

    RoverState& state_;
    int port_;
    std::atomic<bool> running_{false};
    std::thread server_thread_;
};