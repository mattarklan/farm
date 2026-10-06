#pragma once

#include "rover/state/rover_state.hpp"
#include <atomic>
#include <thread>
#include <string>

// Local Cartesian coordinate structure
struct LocalPoint {
    double east{0.0};   // X (meters)
    double north{0.0};  // Y (meters)
    double up{0.0};     // Z (meters)
};

// Origin reference point
struct GeoOrigin {
    double latitude{0.0};
    double longitude{0.0};
    double altitude{0.0};
};

class GpsDriver {
public:
    GpsDriver(RoverState& state, const std::string& mmap_file = "rover_pos.dat");
    ~GpsDriver();

    void start();
    void stop();
    void set_origin(double lat, double lon, double alt);

    // Static WGS84 Geodetic to ENU (East-North-Up) conversion
    static LocalPoint geodetic_to_enu(double lat, double lon, double alt, const GeoOrigin& origin);

private:
    void run();
    bool read_mmap_json(std::string& out_json);

    RoverState& state_;
    std::string mmap_file_;
    std::atomic<bool> running_{false};
    std::thread thread_;

    GeoOrigin origin_;
    bool origin_set_{false};
};