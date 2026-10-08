#include "rover/gps/gps_driver.hpp"
#include <fstream>
#include <sstream>
#include <chrono>
#include <cmath>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GpsDriver::GpsDriver(RoverState& state, const std::string& mmap_file)
    : state_(state), mmap_file_(mmap_file) {}

GpsDriver::~GpsDriver() {
    stop();
}

void GpsDriver::set_origin(double lat, double lon, double alt) {
    origin_.latitude = lat;
    origin_.longitude = lon;
    origin_.altitude = alt;
    origin_set_ = true;
}

void GpsDriver::start() {
    running_ = true;
    thread_ = std::thread(&GpsDriver::run, this);
}

void GpsDriver::stop() {
    if (running_) {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
    }
}

LocalPoint GpsDriver::geodetic_to_enu(double lat, double lon, double alt, const GeoOrigin& origin) {
    const double a = 6378137.0;        
    const double f = 1.0 / 298.257223563; 
    const double e2 = f * (2.0 - f);

    auto deg_to_rad = [](double deg) { return deg * M_PI / 180.0; };

    double phi = deg_to_rad(lat);
    double lambda = deg_to_rad(lon);
    double h = alt;

    double phi0 = deg_to_rad(origin.latitude);
    double lambda0 = deg_to_rad(origin.longitude);
    double h0 = origin.altitude;

    auto geodetic_to_ecef = [&](double p_lat, double p_lon, double p_alt, double& x, double& y, double& z) {
        double s_lat = std::sin(p_lat);
        double c_lat = std::cos(p_lat);
        double N = a / std::sqrt(1.0 - e2 * s_lat * s_lat);

        x = (N + p_alt) * c_lat * std::cos(p_lon);
        y = (N + p_alt) * c_lat * std::sin(p_lon);
        z = (N * (1.0 - e2) + p_alt) * s_lat;
    };

    double x, y, z, x0, y0, z0;
    geodetic_to_ecef(phi, lambda, h, x, y, z);
    geodetic_to_ecef(phi0, lambda0, h0, x0, y0, z0);

    double dx = x - x0;
    double dy = y - y0;
    double dz = z - z0;

    LocalPoint enu;
    enu.east  = -std::sin(lambda0) * dx + std::cos(lambda0) * dy;
    enu.north = -std::sin(phi0) * std::cos(lambda0) * dx - std::sin(phi0) * std::sin(lambda0) * dy + std::cos(phi0) * dz;
    enu.up    =  std::cos(phi0) * std::cos(lambda0) * dx + std::cos(phi0) * std::sin(lambda0) * dy + std::sin(phi0) * dz;

    return enu;
}

bool GpsDriver::read_mmap_json(std::string& out_json) {
    std::ifstream file(mmap_file_, std::ios::binary);
    if (!file.is_open()) return false;

    char buffer[257] = {0};
    file.read(buffer, 256);
    out_json = std::string(buffer);
    return !out_json.empty();
}

static std::string extract_json_field(const std::string& json, const std::string& key) {
    size_t key_pos = json.find("\"" + key + "\"");
    if (key_pos == std::string::npos) return "";

    size_t colon_pos = json.find(':', key_pos);
    if (colon_pos == std::string::npos) return "";

    size_t start_pos = json.find_first_not_of(" \"", colon_pos + 1);
    if (start_pos == std::string::npos) return "";

    size_t end_pos = json.find_first_of(",}\"", start_pos);
    if (end_pos == std::string::npos) return "";

    return json.substr(start_pos, end_pos - start_pos);
}

void GpsDriver::run() {
    if (!origin_set_) {
        set_origin(-32.1504, 116.0123, 15.0);
    }

    while (running_) {
        std::string json_data;

        if (read_mmap_json(json_data)) {
            try {
                std::string lat_str  = extract_json_field(json_data, "lat");
                std::string lon_str  = extract_json_field(json_data, "lon");
                std::string fix_str  = extract_json_field(json_data, "fix");
                std::string sats_str = extract_json_field(json_data, "sats");

                if (!lat_str.empty() && !lon_str.empty()) {
                    double lat = std::stod(lat_str);
                    double lon = std::stod(lon_str);
                    int sats = sats_str.empty() ? 0 : std::stoi(sats_str);
                    bool is_rtk = (fix_str.find("Fixed RTK") != std::string::npos);

                    // Convert Geodetic (Lat/Lon) -> Local ENU Cartesian Frame
                    LocalPoint enu = geodetic_to_enu(lat, lon, origin_.altitude, origin_);

                    // Update shared system state
                    {
                        std::lock_guard<std::mutex> lock(state_.state_mutex);
                        state_.gps.latitude   = lat;
                        state_.gps.longitude  = lon;
                        state_.gps.altitude   = origin_.altitude;
                        state_.gps.rtk_fixed  = is_rtk;
                        state_.gps.sats       = sats;
                        state_.gps.valid      = true;

                        state_.pose.x = enu.east;
                        state_.pose.y = enu.north;
                    }
                }
            } catch (...) {
                // Ignore transient partial read errors
            }
        }

        // Poll shared memory at 5 Hz (200 ms sleep interval)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}