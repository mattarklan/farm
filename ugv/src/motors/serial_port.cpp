#include "rover/motors/serial_port.hpp"
#include <iostream>

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port_name, uint32_t baud_rate) {
#ifdef _WIN32
    // Windows COM ports above COM9 require '\\.\' prefix
    std::string full_port_name = "\\\\.\\" + port_name;

    h_serial_ = CreateFileA(
        full_port_name.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (h_serial_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcb_serial_params = {};
    dcb_serial_params.DCBlength = sizeof(dcb_serial_params);

    if (!GetCommState(h_serial_, &dcb_serial_params)) {
        close();
        return false;
    }

    dcb_serial_params.BaudRate = baud_rate;
    dcb_serial_params.ByteSize = 8;
    dcb_serial_params.StopBits = ONESTOPBIT;
    dcb_serial_params.Parity   = NOPARITY;

    if (!SetCommState(h_serial_, &dcb_serial_params)) {
        close();
        return false;
    }

    // Set non-blocking timeouts for responsiveness
    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout         = 10;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(h_serial_, &timeouts)) {
        close();
        return false;
    }

    is_open_ = true;
    return true;
#else
    return false;
#endif
}

void SerialPort::close() {
#ifdef _WIN32
    if (h_serial_ != INVALID_HANDLE_VALUE) {
        CloseHandle(h_serial_);
        h_serial_ = INVALID_HANDLE_VALUE;
    }
#endif
    is_open_ = false;
}

bool SerialPort::is_open() const {
    return is_open_;
}

bool SerialPort::write_string(const std::string& data) {
    if (!is_open_) return false;

#ifdef _WIN32
    DWORD bytes_written = 0;
    if (!WriteFile(h_serial_, data.c_str(), static_cast<DWORD>(data.size()), &bytes_written, NULL)) {
        return false;
    }
    return bytes_written == data.size();
#else
    return false;
#endif
}

bool SerialPort::read_line(std::string& line) {
    if (!is_open_) return false;

    // Return buffered line if present
    size_t pos = rx_buffer_.find('\n');
    if (pos != std::string::npos) {
        line = rx_buffer_.substr(0, pos);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        rx_buffer_.erase(0, pos + 1);
        return true;
    }

#ifdef _WIN32
    char ch;
    DWORD bytes_read = 0;
    while (ReadFile(h_serial_, &ch, 1, &bytes_read, NULL) && bytes_read > 0) {
        if (ch == '\n') {
            line = rx_buffer_;
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            rx_buffer_.clear();
            return true;
        } else {
            rx_buffer_ += ch;
        }
    }
#endif
    return false;
}