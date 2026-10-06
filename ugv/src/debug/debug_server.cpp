#include "rover/debug/debug_server.hpp"
#include <iostream>
#include <string>
#include <sstream>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef SOCKET socket_t;
    #define INVALID_SOCKET_VAL INVALID_SOCKET
    #define SOCKET_ERR SOCKET_ERROR
    #define CLOSE_SOCKET closesocket
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    typedef int socket_t;
    #define INVALID_SOCKET_VAL -1
    #define SOCKET_ERR -1
    #define CLOSE_SOCKET close
#endif

DebugServer::DebugServer(RoverState& state, int port)
    : state_(state), port_(port) {}

DebugServer::~DebugServer() {
    stop();
}

void DebugServer::start() {
    running_ = true;
    server_thread_ = std::thread(&DebugServer::run, this);
}

void DebugServer::stop() {
    if (running_) {
        running_ = false;
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }
}

void DebugServer::run() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "[DebugServer] WSAStartup failed\n";
        return;
    }
#endif

    socket_t listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd == INVALID_SOCKET_VAL) {
        std::cerr << "[DebugServer] Failed to create socket\n";
        cleanup_winsock();
        return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(listen_fd, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERR) {
        std::cerr << "[DebugServer] Bind failed on port " << port_ << "\n";
        CLOSE_SOCKET(listen_fd);
        cleanup_winsock();
        return;
    }

    if (listen(listen_fd, 1) == SOCKET_ERR) {
        std::cerr << "[DebugServer] Listen failed\n";
        CLOSE_SOCKET(listen_fd);
        cleanup_winsock();
        return;
    }

    std::cout << "[DebugServer] Server listening on port " << port_ << "...\n";

    while (running_) {
        // Non-blocking accept check setup using select
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(listen_fd, &readfds);

        timeval timeout{1, 0}; // 1-second timeout
        int activity = select(static_cast<int>(listen_fd + 1), &readfds, nullptr, nullptr, &timeout);

        if (activity > 0 && FD_ISSET(listen_fd, &readfds)) {
            socket_t client_fd = accept(listen_fd, nullptr, nullptr);
            if (client_fd != INVALID_SOCKET_VAL) {
                std::cout << "[DebugServer] Client connected!\n";

                // Send snapshot telemetry
                std::string snapshot;
                {
                    std::lock_guard<std::mutex> lock(state_.state_mutex);
                    std::ostringstream ss;
                    ss << "STATUS: ONLINE\n"
                       << "Pose X: " << state_.pose.x << " Y: " << state_.pose.y << " Yaw: " << state_.pose.yaw << "\n"
                       << "RTK Fixed: " << (state_.gps.rtk_fixed ? "YES" : "NO") << "\n";
                    snapshot = ss.str();
                }

                send(client_fd, snapshot.c_str(), static_cast<int>(snapshot.size()), 0);

// Graceful shutdown sequence
#ifdef _WIN32
    shutdown(client_fd, SD_SEND);
#else
    shutdown(client_fd, SHUT_WR);
#endif

// Give the client a moment to read before closing
char recv_buf[16];
recv(client_fd, recv_buf, sizeof(recv_buf), 0);

CLOSE_SOCKET(client_fd);
            }
        }
    }

    CLOSE_SOCKET(listen_fd);
    cleanup_winsock();
    std::cout << "[DebugServer] Stopped.\n";
}

void DebugServer::cleanup_winsock() {
#ifdef _WIN32
    WSACleanup();
#endif
}