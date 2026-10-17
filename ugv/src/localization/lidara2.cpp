#include "rover/localization/lidara2.hpp"
#include "rover/comms/udp_bus.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace rover::localization {

LidarA2::LidarA2(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {}

LidarA2::~LidarA2() {
    stop();
}

bool LidarA2::isConnected() {
#ifdef _WIN32
    std::string full_path = "\\\\.\\" + port_;
    HANDLE h = CreateFileA(full_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    CloseHandle(h);
    return true;
#else
    int fd = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return false;
    close(fd);
    return true;
#endif
}

void LidarA2::start() {
    if (running_) return;
    running_ = true;
    worker_thread_ = std::thread(&LidarA2::run, this);
}

void LidarA2::stop() {
    running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

void LidarA2::run() {
    using namespace std::chrono_literals;
    std::cout << "[LidarA2] Connecting to RPLidar on " << port_ << " at " << baudrate_ << " baud...\n";

    // ---------- open + configure the port ----------
#ifdef _WIN32
    std::string full_path = "\\\\.\\" + port_;
    HANDLE hSerial = CreateFileA(full_path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "[LidarA2 ERROR] Failed to open serial port " << port_ << "\n";
        running_ = false;
        return;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = baudrate_;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 20;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(hSerial, &timeouts);
    SetupComm(hSerial, 16384, 4096);

    auto writeBytes = [&](const unsigned char* d, size_t n) {
        DWORD w = 0;
        WriteFile(hSerial, d, static_cast<DWORD>(n), &w, nullptr);
    };
    auto readBytes = [&](unsigned char* d, size_t n) -> int {
        DWORD r = 0;
        if (!ReadFile(hSerial, d, static_cast<DWORD>(n), &r, nullptr)) return 0;
        return static_cast<int>(r);
    };
    auto purge = [&]() { PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR); };
    auto motorOn  = [&]() { EscapeCommFunction(hSerial, CLRDTR); };  // DTR low = motor on (A2 USB adapter)
    auto motorOff = [&]() { EscapeCommFunction(hSerial, SETDTR); };
    auto closePort = [&]() { CloseHandle(hSerial); };
#else
    int fd = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "[LidarA2 ERROR] Failed to open serial port " << port_ << "\n";
        running_ = false;
        return;
    }
    struct termios tty;
    tcgetattr(fd, &tty);
    cfmakeraw(&tty);  // IMPORTANT: no echo / newline translation / canonical mode, or bytes get corrupted
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100 ms read timeout
    tcsetattr(fd, TCSANOW, &tty);

    auto writeBytes = [&](const unsigned char* d, size_t n) { (void)!write(fd, d, n); };
    auto readBytes = [&](unsigned char* d, size_t n) -> int {
        int r = static_cast<int>(read(fd, d, n));
        return r > 0 ? r : 0;
    };
    auto purge = [&]() { tcflush(fd, TCIOFLUSH); };
    auto setDtr = [&](bool high) {
        int s = 0;
        ioctl(fd, TIOCMGET, &s);
        if (high) s |= TIOCM_DTR; else s &= ~TIOCM_DTR;
        ioctl(fd, TIOCMSET, &s);
    };
    auto motorOn  = [&]() { setDtr(false); };
    auto motorOff = [&]() { setDtr(true); };
    auto closePort = [&]() { close(fd); };
#endif

    // ---------- startup sequence (mirrors what the python library does) ----------
    const unsigned char stop_cmd[]  = {0xA5, 0x25};
    const unsigned char start_cmd[] = {0xA5, 0x20};
    // Set motor PWM (newer A2 firmware): A5 F0 02 <pwm_lo> <pwm_hi> <xor checksum>; 660 = 0x0294
    const unsigned char pwm_on[]  = {0xA5, 0xF0, 0x02, 0x94, 0x02, 0xC1};
    const unsigned char pwm_off[] = {0xA5, 0xF0, 0x02, 0x00, 0x00, 0x57};

    writeBytes(stop_cmd, 2);          // make sure nothing is already streaming
    std::this_thread::sleep_for(100ms);
    motorOn();
    writeBytes(pwm_on, sizeof(pwm_on));
    std::cout << "[LidarA2] Spinning up motor...\n";
    std::this_thread::sleep_for(1500ms);  // let the motor reach speed
    purge();                              // drop any stale bytes so we start aligned

    writeBytes(start_cmd, 2);

    // Read the 7-byte response descriptor (A5 5A 05 00 00 40 81)
    unsigned char header[7];
    size_t got = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (got < 7 && running_ && std::chrono::steady_clock::now() - t0 < 2s) {
        got += readBytes(header + got, 7 - got);
    }
    if (got < 7 || header[0] != 0xA5 || header[1] != 0x5A) {
        std::cerr << "[LidarA2 ERROR] Bad/missing scan response descriptor (got " << got << " bytes)\n";
        writeBytes(stop_cmd, 2);
        writeBytes(pwm_off, sizeof(pwm_off));
        motorOff();
        closePort();
        running_ = false;
        return;
    }

    rover::comms::UdpBus udp_bus("239.255.42.42", "127.0.0.1");
    std::vector<std::pair<double, double>> current_scan;
    current_scan.reserve(2000);

    auto publish = [&]() {
        double ts = std::chrono::duration<double>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
        std::ostringstream json;
        json << "{\"type\":\"lidar\",\"v\":1,\"ts\":" << std::fixed << std::setprecision(6) << ts
             << ",\"points\":[";
        for (size_t i = 0; i < current_scan.size(); ++i) {
            json << "[" << std::setprecision(2) << current_scan[i].first
                 << "," << std::setprecision(1) << current_scan[i].second << "]";
            if (i + 1 < current_scan.size()) json << ",";
        }
        json << "]}";
        udp_bus.send(5005, json.str());
    };

    std::vector<unsigned char> rx_buffer;
    rx_buffer.reserve(8192);
    unsigned char chunk[512];

    // ---------- main loop ----------
    while (running_) {
        int n = readBytes(chunk, sizeof(chunk));
        if (n <= 0) {
            std::this_thread::sleep_for(1ms);
            continue;
        }
        rx_buffer.insert(rx_buffer.end(), chunk, chunk + n);

        size_t pos = 0;
        while (rx_buffer.size() - pos >= 5) {
            const unsigned char b0 = rx_buffer[pos];
            const unsigned char b1 = rx_buffer[pos + 1];

            const bool s_bit     = b0 & 0x01;         // start of a new revolution
            const bool s_inv_bit = (b0 >> 1) & 0x01;  // must be the inverse of s_bit
            const bool c_bit     = b1 & 0x01;         // check bit, always 1

            if (s_bit == s_inv_bit || !c_bit) {  // not a valid packet start: resync by 1 byte
                ++pos;
                continue;
            }

            const unsigned char b2 = rx_buffer[pos + 2];
            const unsigned char b3 = rx_buffer[pos + 3];
            const unsigned char b4 = rx_buffer[pos + 4];

            // Angle: byte1 bits 7..1 are the LOW 7 bits (bit 0 is the check bit), byte2 is the HIGH 8 bits.
            const double angle_deg = static_cast<double>((b2 << 7) | (b1 >> 1)) / 64.0;
            const double dist_mm   = static_cast<double>((b4 << 8) | b3) / 4.0;

            // New revolution flag => flush previous scan (same as python iter_scans)
            if (s_bit) {
                if (current_scan.size() >= 20) publish();
                current_scan.clear();
            }

            if (dist_mm > 0.0 && dist_mm <= 12000.0) {
                current_scan.push_back({angle_deg, dist_mm});
            }
            pos += 5;
        }
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + pos);

        if (rx_buffer.size() > 65536) rx_buffer.clear();  // safety against runaway growth
    }

    // ---------- shutdown ----------
    writeBytes(stop_cmd, 2);
    std::this_thread::sleep_for(50ms);
    writeBytes(pwm_off, sizeof(pwm_off));
    motorOff();
    closePort();
}

} // namespace rover::localization