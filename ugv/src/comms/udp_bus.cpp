#include "rover/comms/udp_bus.hpp"
#include <iostream>
#include <sstream>

namespace rover::comms {

UdpBus::UdpBus(const std::string& group_ip, const std::string& interface_ip)
    : group_ip_(group_ip), interface_ip_(interface_ip), send_socket_(-1), recv_socket_(-1) 
{
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    // Setup Sender Socket
    send_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int ttl = 2;
    setsockopt(send_socket_, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl));

    // Setup Receiver Socket (Multicast listener)
    recv_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    
    // Allow multiple bindings to the same multicast port
    int reuse = 1;
    setsockopt(recv_socket_, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse));

    // Make receiver non-blocking so poll_messages() won't hang
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(recv_socket_, FIONBIO, &mode);
#else
    int flags = fcntl(recv_socket_, F_GETFL, 0);
    fcntl(recv_socket_, F_SETFL, flags | O_NONBLOCK);
#endif
}

UdpBus::~UdpBus() {
#ifdef _WIN32
    if (send_socket_ != INVALID_SOCKET) closesocket(send_socket_);
    if (recv_socket_ != INVALID_SOCKET) closesocket(recv_socket_);
    WSACleanup();
#else
    if (send_socket_ >= 0) close(send_socket_);
    if (recv_socket_ >= 0) close(recv_socket_);
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

    while (true) {
        int bytes_received = recvfrom(recv_socket_, buffer, sizeof(buffer) - 1, 0,
                                     (struct sockaddr*)&src_addr, &addr_len);
        if (bytes_received <= 0) break;

        buffer[bytes_received] = '\0';
        messages.push_back(std::string(buffer));
    }
    return messages;
}

} // namespace rover::comms