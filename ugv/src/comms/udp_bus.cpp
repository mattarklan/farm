#include "rover/comms/udp_bus.hpp"
#include <iostream>
#include <sstream>
#include <vector>

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <netdb.h>
#endif

namespace rover::comms {

UdpBus::UdpBus(const std::string& group_ip, const std::string& interface_ip)
    : send_socket_(-1), group_ip_(group_ip), interface_ip_(interface_ip) 
{
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    // Setup Sender Socket
    send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int ttl = 2;
    setsockopt(send_socket_, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl));

    // Listen on protocol ports: 5006 (esp/cmd_vel), 5007 (rtk), 5008 (pose/goal/path)
    std::vector<int> ports_to_listen = {5006, 5007, 5008};

    for (int port : ports_to_listen) {
#ifdef _WIN32
        SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET) continue;
#else
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) continue;
#endif

        int reuse = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse));

        // Bind to INADDR_ANY to reliably receive local multicast packets on Windows
        sockaddr_in local_addr{};
        local_addr.sin_family = AF_INET;
        local_addr.sin_port = htons(port);
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            std::cerr << "[UdpBus] Failed to bind receiver socket to port " << port << "!\n";
        }

        // JOIN MULTICAST GROUP and enable loopback
        ip_mreq mreq{};
        mreq.imr_multiaddr.s_addr = inet_addr(group_ip_.c_str());
        mreq.imr_interface.s_addr = inet_addr(interface_ip_.c_str());
        if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0) {
            std::cerr << "[UdpBus] Failed to join multicast group on port " << port << "!\n";
        }

        int loop = 1;
        setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, (char*)&loop, sizeof(loop));

        // Make receiver non-blocking
#ifdef _WIN32
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);
#else
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
#endif

        recv_sockets_.push_back(sock);
    }
}

UdpBus::~UdpBus() {
#ifdef _WIN32
    if (send_socket_ != INVALID_SOCKET) closesocket(send_socket_);
    for (SOCKET sock : recv_sockets_) {
        if (sock != INVALID_SOCKET) closesocket(sock);
    }
    WSACleanup();
#else
    if (send_socket_ >= 0) close(send_socket_);
    for (int sock : recv_sockets_) {
        if (sock >= 0) close(sock);
    }
#endif
}

bool UdpBus::send(int port, const std::string& json_payload) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    dest_addr.sin_addr.s_addr = inet_addr(group_ip_.c_str());

    int bytes_sent = sendto(send_socket_, json_payload.c_str(), static_cast<int>(json_payload.size()), 0,
                            (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    return bytes_sent != -1;
}

std::vector<std::string> UdpBus::poll_messages() {
    std::vector<std::string> messages;
    char buffer[65535];
    sockaddr_in src_addr{};
    socklen_t addr_len = sizeof(src_addr);

    for (auto sock : recv_sockets_) {
        while (true) {
            int bytes_received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                                       (struct sockaddr*)&src_addr, &addr_len);
            if (bytes_received <= 0) break;

            buffer[bytes_received] = '\0';
            messages.push_back(std::string(buffer));
        }
    }
    return messages;
}

} // namespace rover::comms