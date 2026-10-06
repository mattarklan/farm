#pragma once
#include <string>
#include <cstdint>

#ifdef _WIN32
    #include <windows.h>
#endif

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& port_name, uint32_t baud_rate = 115200);
    void close();
    bool is_open() const;

    bool write_string(const std::string& data);
    bool read_line(std::string& line);

private:
    #ifdef _WIN32
        HANDLE h_serial_{INVALID_HANDLE_VALUE};
    #else
        int fd_{-1};
    #endif
    bool is_open_{false};
    std::string rx_buffer_;
};