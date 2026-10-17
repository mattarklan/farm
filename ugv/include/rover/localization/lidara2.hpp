#pragma once

#include <string>
#include <atomic>
#include <thread>

namespace rover::localization {

class LidarA2 {
public:
    explicit LidarA2(const std::string& port = "COM3", int baudrate = 115200);
    ~LidarA2();

    bool isConnected();
    void start();
    void stop();

private:
    void run();

    std::string port_;
    int baudrate_;
    std::atomic<bool> running_{false};
    std::thread worker_thread_;
};

} // namespace rover::localization