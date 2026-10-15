#include <winsock2.h>
#pragma once

#include <string>
#include <vector>
#include <mutex>

#ifdef _WIN32

#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

namespace rover::comms {

class UdpBus {
public:
    UdpBus(const std::string& group_ip = "239.255.42.42", const std::string& interface_ip = "127.0.0.1");
    ~UdpBus();

    bool send(int port, const std::string& json_payload);
    std::vector<std::string> poll_messages();

private:
#ifdef _WIN32
    SOCKET send_socket_;
    std::vector<SOCKET> recv_sockets_;
#else
    int send_socket_;
    std::vector<int> recv_sockets_;
#endif
    std::string group_ip_;
    std::string interface_ip_;
    std::mutex bus_mutex_;
};

} // namespace rover::comms