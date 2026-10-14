#include "rover/planning/lcostmap.hpp"
#include "rover/state/rover_state.hpp"
#include "rover/gps/gps_driver.hpp"
#include "rover/motors/motor_interface.hpp"
#include "rover/debug/debug_server.hpp"
#include "rover/localization/pose_estimator.hpp"
#include "rover/planning/theta_star.hpp"
#include "rover/planning/follower.hpp"
#include "rover/comms/udp_bus.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <string>

// Helper to update rtk/rover_pos.dat with live JSON pose telemetry
void update_rover_pos_file(const std::string& filepath, const Pose& pose) {
    std::ofstream outfile(filepath, std::ios::trunc);
    if (outfile.is_open()) {
        auto now = std::chrono::system_clock::now();
        double ts = std::chrono::duration<double>(now.time_since_epoch()).count();

        outfile << std::fixed << std::setprecision(9)
                << "{\"lat\": -31.950500067"
                << ", \"lon\": 115.860489505"
                << ", \"fix\": \"Fixed RTK\""
                << ", \"sats\": 23"
                << ", \"hdop\": 0.67"
                << ", \"corr_age\": 0.8"
                << ", \"ts\": " << std::setprecision(7) << ts
                << ", \"enu_x\": " << std::setprecision(2) << pose.x
                << ", \"enu_y\": " << pose.y
                << ", \"yaw\": " << std::setprecision(4) << pose.yaw
                << "}\n";
    }
}

// Helper to get current unix timestamp for UGV protocol messages
double get_timestamp() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

// Robust helper function to extract numeric values from JSON strings without substring offset bugs
double extract_json_val(const std::string& json_str, const std::string& key) {
    std::string search_key = "\"" + key + "\"";
    size_t key_pos = json_str.find(search_key);
    if (key_pos == std::string::npos) return 0.0;

    size_t colon_pos = json_str.find(':', key_pos);
    if (colon_pos == std::string::npos) return 0.0;

    size_t val_start = colon_pos + 1;
    while (val_start < json_str.size() && (json_str[val_start] == ' ' || json_str[val_start] == '\t' || json_str[val_start] == '\"')) {
        val_start++;
    }

    if (val_start >= json_str.size()) return 0.0;

    try {
        return std::stod(json_str.substr(val_start));
    } catch (...) {
        return 0.0;
    }
}

// Persistent, world-anchored occupancy map built up from LiDAR scans (log-odds per cell).
// Unlike the old per-scan "synthetic grid" this is never cleared, so the planner remembers
// every wall the rover has seen, and cells stay fixed in the world as the rover moves.
class OccupancyMap {
public:
    OccupancyMap(int w, int h, double res) : w_(w), h_(h), res_(res), lo_(w * h, 0) {}

    bool initialised() const { return init_; }
    void init(double cx, double cy) {
        ox_ = cx - w_ * res_ / 2.0;
        oy_ = cy - h_ * res_ / 2.0;
        init_ = true;
    }
    double originX() const { return ox_; }
    double originY() const { return oy_; }

    bool toCell(double wx, double wy, int& cx, int& cy) const {
        cx = static_cast<int>(std::floor((wx - ox_) / res_));
        cy = static_cast<int>(std::floor((wy - oy_) / res_));
        return inBounds(cx, cy);
    }

    // scans: {range_m, body_angle_rad}
    void integrate(double rx, double ry, double ryaw, const std::vector<std::pair<double, double>>& scans) {
        int sx, sy;
        if (!init_ || !toCell(rx, ry, sx, sy)) return;

        for (const auto& s : scans) {
            double hx = rx + s.first * std::cos(s.second + ryaw);
            double hy = ry + s.first * std::sin(s.second + ryaw);
            int ex = static_cast<int>(std::floor((hx - ox_) / res_));
            int ey = static_cast<int>(std::floor((hy - oy_) / res_));

            traceFree(sx, sy, ex, ey);
            if (inBounds(ex, ey)) bump(ex, ey, +5);
        }
    }

    // Planning grid: 0 = free, 1 = inflated (robot body would touch), 2 = lethal.
    // Cells within clear_cells of the rover are forced free (unless truly lethal) so the planner
    // can always escape when the rover is already close to a wall.
    std::vector<uint8_t> buildPlanningGrid(double rx, double ry, int inflate_cells, int clear_cells) const {
        std::vector<uint8_t> g(w_ * h_, 0);
        const int r2 = inflate_cells * inflate_cells;

        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                if (lo_[y * w_ + x] < kOccThresh) continue;
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        if (dx * dx + dy * dy > r2) continue;
                        int nx = x + dx, ny = y + dy;
                        if (!inBounds(nx, ny)) continue;
                        uint8_t& c = g[ny * w_ + nx];
                        if (c == 0) c = 1;
                    }
                }
            }
        }
        for (int i = 0; i < w_ * h_; ++i) {
            if (lo_[i] >= kOccThresh) g[i] = 2;
        }

        int rcx, rcy;
        if (toCell(rx, ry, rcx, rcy)) {
            const int c2 = clear_cells * clear_cells;
            for (int dy = -clear_cells; dy <= clear_cells; ++dy) {
                for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
                    if (dx * dx + dy * dy > c2) continue;
                    int nx = rcx + dx, ny = rcy + dy;
                    if (inBounds(nx, ny) && g[ny * w_ + nx] == 1) g[ny * w_ + nx] = 0;
                }
            }
        }
        return g;
    }

    // ---------------------------------------------------------------- persistence
    bool consumeDirty() { bool d = dirty_; dirty_ = false; return d; }

    // Lat/lon of map (0,0). Stored in the file so a map made for a different origin is never used.
    void setReference(double lat, double lon) { ref_lat_ = lat; ref_lon_ = lon; }

    // Binary format (v2): "RMAP", int32 version, int32 w, int32 h, double res, double origin_x, double origin_y,
    // double ref_lat, double ref_lon (lat/lon of map 0,0),
    // then w*h int8 log-odds (row-major, y = 0 is the southern edge). Native endianness.
    // Written to a temp file and renamed, so a crash mid-save can't corrupt the previous map.
    // Also writes a north-up PGM preview you can open in any image viewer.
    bool save(const std::string& path, const std::string& pgm_path) {
        if (!init_) return false;
        const std::string tmp = path + ".tmp";
        {
            std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
            if (!f) { std::cerr << "[Map] Cannot write " << tmp << "\n"; return false; }
            const char magic[4] = {'R', 'M', 'A', 'P'};
            const int32_t ver = 2, w = w_, h = h_;
            f.write(magic, 4);
            f.write(reinterpret_cast<const char*>(&ver), sizeof(ver));
            f.write(reinterpret_cast<const char*>(&w), sizeof(w));
            f.write(reinterpret_cast<const char*>(&h), sizeof(h));
            f.write(reinterpret_cast<const char*>(&res_), sizeof(res_));
            f.write(reinterpret_cast<const char*>(&ox_), sizeof(ox_));
            f.write(reinterpret_cast<const char*>(&oy_), sizeof(oy_));
            f.write(reinterpret_cast<const char*>(&ref_lat_), sizeof(ref_lat_));
            f.write(reinterpret_cast<const char*>(&ref_lon_), sizeof(ref_lon_));
            f.write(reinterpret_cast<const char*>(lo_.data()), static_cast<std::streamsize>(lo_.size()));
            if (!f) { std::cerr << "[Map] Write failed for " << tmp << "\n"; return false; }
        }
        std::error_code ec;
        std::filesystem::rename(tmp, path, ec); // replaces an existing file
        if (ec) { std::cerr << "[Map] Cannot replace " << path << ": " << ec.message() << "\n"; return false; }

        if (!pgm_path.empty()) {
            std::ofstream p(pgm_path, std::ios::binary | std::ios::trunc);
            if (p) {
                p << "P5\n" << w_ << " " << h_ << "\n255\n";
                std::vector<unsigned char> row(w_);
                for (int y = h_ - 1; y >= 0; --y) { // north at the top
                    for (int x = 0; x < w_; ++x) {
                        int v = lo_[y * w_ + x];
                        row[x] = (v >= kOccThresh) ? 0 : (v > 0 ? 90 : (v < 0 ? 255 : 160));
                    }
                    p.write(reinterpret_cast<const char*>(row.data()), w_);
                }
            }
        }
        dirty_ = false;
        return true;
    }

    // Returns false (and prints why) if the file is missing, unreadable or doesn't match this map's size/resolution.
    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        char magic[4];
        int32_t ver = 0, w = 0, h = 0;
        double res = 0, ox = 0, oy = 0, rlat = 0, rlon = 0;
        f.read(magic, 4);
        f.read(reinterpret_cast<char*>(&ver), sizeof(ver));
        f.read(reinterpret_cast<char*>(&w), sizeof(w));
        f.read(reinterpret_cast<char*>(&h), sizeof(h));
        f.read(reinterpret_cast<char*>(&res), sizeof(res));
        f.read(reinterpret_cast<char*>(&ox), sizeof(ox));
        f.read(reinterpret_cast<char*>(&oy), sizeof(oy));
        f.read(reinterpret_cast<char*>(&rlat), sizeof(rlat));
        f.read(reinterpret_cast<char*>(&rlon), sizeof(rlon));
        if (!f || std::string(magic, 4) != "RMAP" || ver != 2) {
            std::cerr << "[Map] " << path << " is not a valid map file.\n";
            return false;
        }
        if (w != w_ || h != h_ || std::abs(res - res_) > 1e-9) {
            std::cerr << "[Map] " << path << " is " << w << "x" << h << " @ " << res << " m but this build is "
                      << w_ << "x" << h_ << " @ " << res_ << " m.\n";
            return false;
        }
        if (std::abs(rlat - ref_lat_) > 1e-7 || std::abs(rlon - ref_lon_) > 1e-7) { // ~1 cm
            std::cerr << "[Map] " << path << " was made for map origin (" << std::fixed << std::setprecision(9)
                      << rlat << ", " << rlon << ") but the current origin is (" << ref_lat_ << ", " << ref_lon_
                      << "). Not using it.\n";
            return false;
        }
        std::vector<int8_t> data(static_cast<size_t>(w_) * h_);
        f.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
        if (!f) {
            std::cerr << "[Map] " << path << " is truncated.\n";
            return false;
        }
        lo_ = std::move(data);
        ox_ = ox;
        oy_ = oy;
        init_ = true;
        dirty_ = false;
        return true;
    }

    // ---------------------------------------------------------------- localisation
    struct MatchResult {
        bool ok{false};
        double dx{0.0}, dy{0.0}, dyaw{0.0}; // correction to ADD to the pose estimate
        double score{0.0}, base{0.0};
        int beams{0};
    };

    // Correlative scan matcher: tries small pose offsets around the estimate and keeps the one whose beam
    // endpoints land on the most occupied cells of the map. Cells next to an occupied cell earn partial credit,
    // which gives a smooth score and sub-cell behaviour. ok=false when the map there is too empty to trust
    // (e.g. exploring somewhere new) or the offset doesn't clearly beat the current estimate.
    MatchResult matchScan(double rx, double ry, double ryaw,
                          const std::vector<std::pair<double, double>>& scans,
                          double lin_win_m, double ang_win_rad) const {
        MatchResult out;
        if (!init_ || scans.empty()) return out;

        const size_t stride = std::max<size_t>(1, scans.size() / 120);
        std::vector<std::pair<double, double>> beams;
        for (size_t i = 0; i < scans.size(); i += stride) beams.push_back(scans[i]);
        const int n = static_cast<int>(beams.size());
        out.beams = n;

        const double lin_step = std::max(res_ * 0.5, 0.05);
        const double ang_step = 1.5 * M_PI / 180.0;
        const int nl = static_cast<int>(std::round(lin_win_m / lin_step));
        const int na = static_cast<int>(std::round(ang_win_rad / ang_step));

        auto scoreAt = [&](double x, double y, const std::vector<std::pair<double, double>>& offs) {
            double s = 0.0;
            for (const auto& o : offs) {
                int cx = static_cast<int>(std::floor((x + o.first - ox_) / res_));
                int cy = static_cast<int>(std::floor((y + o.second - oy_) / res_));
                if (!inBounds(cx, cy)) continue;
                if (lo_[cy * w_ + cx] >= kOccThresh) { s += 1.0; continue; }
                bool adjacent = false; // (not "near": that is a macro in the Windows headers)
                for (int dy = -1; dy <= 1 && !adjacent; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        int nx = cx + dx, ny = cy + dy;
                        if (inBounds(nx, ny) && lo_[ny * w_ + nx] >= kOccThresh) { adjacent = true; break; }
                    }
                if (adjacent) s += 0.4;
            }
            return s;
        };

        auto offsetsFor = [&](double yaw) {
            std::vector<std::pair<double, double>> offs;
            offs.reserve(beams.size());
            for (const auto& b : beams)
                offs.push_back({b.first * std::cos(b.second + yaw), b.first * std::sin(b.second + yaw)});
            return offs;
        };

        out.base = scoreAt(rx, ry, offsetsFor(ryaw));

        double best_adj = -1e18, best_raw = 0.0, bdx = 0.0, bdy = 0.0, bdyaw = 0.0;
        for (int ia = -na; ia <= na; ++ia) {
            const double dyaw = ia * ang_step;
            const auto offs = offsetsFor(ryaw + dyaw);
            for (int ix = -nl; ix <= nl; ++ix) {
                for (int iy = -nl; iy <= nl; ++iy) {
                    const double dx = ix * lin_step, dy = iy * lin_step;
                    const double raw = scoreAt(rx + dx, ry + dy, offs);
                    // prefer the smallest correction when scores are close
                    const double pen = 0.05 * n * (std::hypot(dx, dy) / lin_win_m + std::abs(dyaw) / ang_win_rad);
                    if (raw - pen > best_adj) {
                        best_adj = raw - pen;
                        best_raw = raw;
                        bdx = dx; bdy = dy; bdyaw = dyaw;
                    }
                }
            }
        }

        const bool moved = (bdx != 0.0 || bdy != 0.0 || bdyaw != 0.0);
        out.score = best_raw;
        out.ok = (best_raw >= 0.25 * n) && (!moved || best_raw - out.base >= 0.03 * n);
        if (out.ok) { out.dx = bdx; out.dy = bdy; out.dyaw = bdyaw; }
        return out;
    }

    // Is the remaining path (from the point nearest the rover onwards) still clear on the live map?
    // Cheap: samples the polyline once per cell and looks for occupied cells within check_m.
    // Samples within near_m of the rover only count if they hit an occupied cell directly, matching
    // the "escape disc" the planner uses, so being close to a wall doesn't count as blocked.
    bool pathBlocked(const std::vector<Pose>& path, double rx, double ry, double check_m, double near_m) const {
        if (!init_ || path.size() < 2) return false;

        std::vector<std::pair<double, double>> pts;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            double ax = path[i].x, ay = path[i].y, bx = path[i + 1].x, by = path[i + 1].y;
            double len = std::hypot(bx - ax, by - ay);
            int n = std::max(1, static_cast<int>(std::ceil(len / res_)));
            for (int k = 0; k < n; ++k) {
                double t = static_cast<double>(k) / n;
                pts.push_back({ax + t * (bx - ax), ay + t * (by - ay)});
            }
        }
        pts.push_back({path.back().x, path.back().y});

        size_t start = 0;
        double best = 1e18;
        for (size_t i = 0; i < pts.size(); ++i) {
            double d = std::hypot(pts[i].first - rx, pts[i].second - ry);
            if (d < best) { best = d; start = i; }
        }

        const int check_cells = static_cast<int>(std::round(check_m / res_));
        for (size_t i = start; i < pts.size(); ++i) {
            int cx, cy;
            if (!toCell(pts[i].first, pts[i].second, cx, cy)) continue;
            double dr = std::hypot(pts[i].first - rx, pts[i].second - ry);
            int r = (dr < near_m) ? 0 : check_cells;
            const int r2 = r * r;
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    if (dx * dx + dy * dy > r2) continue;
                    int nx = cx + dx, ny = cy + dy;
                    if (inBounds(nx, ny) && lo_[ny * w_ + nx] >= kOccThresh) return true;
                }
            }
        }
        return false;
    }

private:
    static constexpr int kOccThresh = 3;

    bool inBounds(int x, int y) const { return x >= 0 && x < w_ && y >= 0 && y < h_; }

    void bump(int x, int y, int d) {
        dirty_ = true;
        int8_t& v = lo_[y * w_ + x];
        int nv = static_cast<int>(v) + d;
        v = static_cast<int8_t>(std::clamp(nv, -10, 30));
    }

    // Mark every cell strictly between (x0,y0) and (x1,y1) as seen-free
    void traceFree(int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (!(x == x1 && y == y1)) {
            if (inBounds(x, y)) bump(x, y, -1);
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }
    }

    int w_, h_;
    double res_;
    double ox_{0.0}, oy_{0.0};
    bool init_{false};
    bool dirty_{false};
    double ref_lat_{0.0}, ref_lon_{0.0};
    std::vector<int8_t> lo_;
};

// Set by Ctrl+C / SIGTERM so the main loop can exit cleanly and save the map
static volatile std::sig_atomic_t g_quit = 0;
static void on_signal(int) { g_quit = 1; }

static double wrap_pi(double a) {
    return a - 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
}

// Shortest distance from (x,y) to the path polyline
static double dist_to_path(const std::vector<Pose>& p, double x, double y) {
    if (p.empty()) return 0.0;
    if (p.size() == 1) return std::hypot(p[0].x - x, p[0].y - y);
    double best = 1e18;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        double ax = p[i].x, ay = p[i].y;
        double dx = p[i + 1].x - ax, dy = p[i + 1].y - ay;
        double L2 = dx * dx + dy * dy;
        double t = (L2 == 0.0) ? 0.0 : std::clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0);
        best = std::min(best, std::hypot(x - (ax + t * dx), y - (ay + t * dy)));
    }
    return best;
}

int main() {
    std::cout << "===========================================\n";
    std::cout << "        Rover Navigation System Core       \n";
    std::cout << "===========================================\n" << std::flush;

    RoverState state;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // 1. Initialize Mock Motor Driver, Debug HTTP Server, & UGV UDP Multicast Bus
    MotorInterface motor_driver(state, "COM4", 115200, "../sims/sim_esp.dat");
    motor_driver.start();

    DebugServer debug_server(state, 5000);
    debug_server.start();

    // Multicast group 239.255.42.42 on localhost (127.0.0.1)
    rover::comms::UdpBus udp_bus("239.255.42.42", "127.0.0.1");

    // 2. Initialize Navigation & Pose Estimator
    rover::localization::PoseEstimator pose_estimator;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Capture starting pose from state
    Pose start_pose;
    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.gps.rtk_fixed = false;
        start_pose = state.pose;
    }

    pose_estimator.set_pose(start_pose.x, start_pose.y, start_pose.yaw);

    // 3. Initialize Planner, PurePursuit Follower, and Control States
    std::vector<Pose> global_path;
    
    // PurePursuit controller matching simulation (max_speed=0.8, max_turn=1.0, lookahead=0.6, goal_tol=0.25)
    rover::planning::PurePursuit follower(0.8, 1.0, 0.6, 0.25);

    bool is_teleop_active = false;
    bool has_active_goal = false;

    // Persistent world-anchored occupancy map for the global planner
    // (400x400 cells at 0.10 m = 40 m x 40 m, origin fixed on the first scan/goal; bump if your world is bigger)
    // Anchored on the map centre (0,0) = the visualiser's frame, so the whole world is covered
    // wherever the rover starts. 500 x 500 cells at 0.10 m = 50 m x 50 m.
    const int map_w = 500;
    const int map_h = 500;
    const double res = 0.10;
    const double kInflateM = 0.45; // robot half-width + safety margin (tune to your rover)
    const double kClearM   = 0.35; // free "escape" disc around the rover when planning
    OccupancyMap world_map(map_w, map_h, res);

    // Saved global map: loaded at startup, autosaved while the rover runs, and saved again on Ctrl+C.
    // The map is only valid for the SAME map origin/reference point and size/resolution as when it was saved.
    // Lat/lon of map (0,0). TEST VALUES for now (same as the sim). Must match ORIGIN_LAT/ORIGIN_LON in the
    // visualiser's common/config.py, and must not change once you've saved a map you want to keep.
    const double kOriginLat = -31.950500067;
    const double kOriginLon = 115.860489505;
    world_map.setReference(kOriginLat, kOriginLon);
    bool map_loaded = false;

    const std::string kMapFile     = "global.map";
    const std::string kPreviewFile = "global_preview.pgm"; // north-up image of the map, for viewing
    const double      kSaveEveryS  = 30.0;
    if (std::filesystem::exists(kMapFile)) {
        if (world_map.load(kMapFile)) {
            map_loaded = true;
            std::cout << "[Map] Loaded saved global map from " << kMapFile << " (" << map_w << "x" << map_h
                      << " cells @ " << res << " m).\n";
        } else {
            std::error_code ec;
            std::filesystem::rename(kMapFile, kMapFile + ".bak", ec);
            std::cerr << "[Map] Could not use " << kMapFile << "; kept it as " << kMapFile
                      << ".bak and starting a fresh map.\n";
        }
    }

    // Local Costmap: short-range safety layer only (200x200 cells at 0.05 m = 10 m x 10 m window)
    const double local_res = 0.05;
    LocalCostmap local_costmap(200, 200, local_res);

    rover::planning::ThetaStarPlanner planner(map_w, map_h, res);

    Pose active_goal{0.0, 0.0, 0.0};
    auto last_replan_time = std::chrono::steady_clock::now();
    double forward_clearance = 99.0; // free distance straight ahead (m), updated on every LiDAR scan

    // --- Event-driven replanning (no fixed-rate timer) ---
    // A replan is requested only when something is wrong: the path became blocked, the rover left the
    // path, forward motion has been held by the safety layer, or a slow backstop expires.
    const double kOffPathM        = 1.0;  // replan if the rover is further than this from the path
    const double kBackstopS       = 8.0;  // replan at least this often (catches bugs + better routes)
    const double kCooldownS       = 0.5;  // never replan more often than this
    const double kStallS          = 1.5;  // forward motion held this long -> replan
    const int    kBlockedScans    = 3;    // path must look blocked this many scans in a row (hysteresis)
    const int    kMaxReplanFails  = 3;    // give up on the goal after this many failed replans in a row
    const char*  replan_reason    = nullptr;
    int          blocked_scans    = 0;
    int          replan_fails     = 0;
    bool         stalled          = false;
    auto         stall_since      = std::chrono::steady_clock::now();
    auto request_replan = [&](const char* why) { if (!replan_reason) replan_reason = why; };

    // --- Localisation against the global map ---
    // Scan-matches each LiDAR scan against the map and keeps a small, bounded correction (loc_off_*) that is
    // added to every incoming pose. NOTE: if the visualiser also corrects the pose with its own lidar matcher
    // (it does by default), run it with --no-localise while testing this, or set kLocalise = false.
    const bool   kLocalise      = true;
    const double kLocWinM       = 0.4;   // search +-0.4 m
    const double kLocWinDeg     = 6.0;   // search +-6 deg
    const double kLocGain       = 0.3;   // apply 30% of each match (smooth, rejects single bad scans)
    const double kLocMaxOffM    = 3.0;   // if the accumulated correction exceeds this, something is wrong: reset it
    const double kLocMaxOffDeg  = 30.0;
    double loc_off_x = 0.0, loc_off_y = 0.0, loc_off_yaw = 0.0;
    int    loc_accepts = 0, loc_rejects = 0;
    bool   got_first_scan = false;

    // Correction already applied by relocalisation; the 3 m / 30 deg guard is measured from here
    double loc_base_x = 0.0, loc_base_y = 0.0, loc_base_yaw = 0.0;

    // Latest RTK position in map coordinates (same flat-earth maths as the visualiser)
    double gps_x = 0.0, gps_y = 0.0;
    bool   gps_good = false;                       // Fixed or Float RTK
    auto   gps_time = std::chrono::steady_clock::now();
    bool   pose_seen = false;                      // have we received at least one pose message?
    auto   last_gps_warn = std::chrono::steady_clock::now() - std::chrono::seconds(60);

    // One-shot full-circle heading search against a LOADED map (GPS gives position, scan matching gives heading)
    bool reloc_done = !map_loaded;
    int  reloc_tries = 0;

    // Plan from 'from' to 'goal' on the persistent map, hand the path to the follower and publish it.
    // Returns false (and leaves the current path untouched) if no path exists.
    auto plan_and_publish = [&](const Pose& from, const Pose& goal, bool log) -> bool {
        if (!world_map.initialised()) world_map.init(0.0, 0.0);
        planner.set_origin(world_map.originX(), world_map.originY());

        auto plan_grid = world_map.buildPlanningGrid(
            from.x, from.y,
            static_cast<int>(std::round(kInflateM / res)),
            static_cast<int>(std::round(kClearM / res)));

        auto path = planner.plan_path(from, goal, plan_grid);
        if (path.empty()) return false;

        global_path = path;
        follower.set_path(global_path);
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.global_path = global_path;
        }

        std::ostringstream path_json;
        path_json << "{\"type\":\"path\",\"v\":1,\"ts\": " << get_timestamp() << ",\"points\":[";
        for (size_t i = 0; i < global_path.size(); ++i) {
            path_json << "[" << global_path[i].x << "," << global_path[i].y << "]";
            if (i + 1 < global_path.size()) path_json << ",";
        }
        path_json << "]}";
        udp_bus.send(5008, path_json.str());
        if (log) std::cout << "[Planner] Path published! Total Waypoints: " << global_path.size() << "\n";
        return true;
    };

    {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.global_path = global_path;
    }

    std::cout << "\n[Main] Rover online. Waiting for commands from visualiser...\n\n" << std::flush;

    auto last_loop_time = std::chrono::steady_clock::now();
    auto last_log_time = std::chrono::steady_clock::now();
    auto last_save_time = std::chrono::steady_clock::now();

    while (!g_quit) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_loop_time).count() / 1000000.0;
        last_loop_time = now;

        if (dt > 0.1) dt = 0.05; // Clamp delta time spike on startup

        // --- LISTEN FOR INCOMING BUS MESSAGES (Poses, Goals, LiDAR, Teleop Commands) ---
        auto incoming = udp_bus.poll_messages();
        for (const auto& msg : incoming) {
            
            // A. Check for explicit manual Teleop keypress actions ONLY (prevents cmd_vel loop traps)
            if (msg.find("\"action\":\"teleop\"") != std::string::npos) {
                is_teleop_active = true;
                has_active_goal = false;
                global_path.clear();
                follower.set_path({});
                continue;
            }

            // B. Handle goal cancellation requests from visualiser
            if (msg.find("\"action\":\"cancel\"") != std::string::npos) {
                std::cout << "[Bus] Received goal cancel command from visualiser.\n";
                global_path.clear();
                has_active_goal = false;
                is_teleop_active = false;
                follower.set_path({});
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.velocity = {0.0, 0.0};
                    state.motor_command.linear = 0.0;
                    state.motor_command.angular = 0.0;
                    state.motor_command.enabled = false;
                }
                continue;
            }

            // C. Handle incoming goal commands ("action":"go")
            if (msg.find("\"action\":\"go\"") != std::string::npos || (msg.find("go") != std::string::npos && msg.find("x") != std::string::npos && msg.find("action") != std::string::npos)) {
                double gx = extract_json_val(msg, "x");
                double gy = extract_json_val(msg, "y");

                Pose current_pose;
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    current_pose = state.pose;
                }

                Pose goal_pose{gx, gy, 0.0};
                std::cout << "[Planner] New goal received: (" << gx << ", " << gy << "). Planning path from current pose (" << current_pose.x << ", " << current_pose.y << ")...\n";

                // Plan on the persistent, inflated map. NO straight-line fallback any more: that
                // fallback is what drove the rover into walls whenever planning failed.
                active_goal = goal_pose;
                if (!plan_and_publish(current_pose, goal_pose, true)) {
                    std::cerr << "[Planner] No path to goal found. Goal ignored.\n";
                    has_active_goal = false;
                    global_path.clear();
                    follower.set_path({});
                    continue;
                }

                has_active_goal = true;
                is_teleop_active = false; // Explicitly drop teleop lock when a goal is clicked
                last_replan_time = now;
                replan_reason = nullptr;
                blocked_scans = 0;
                replan_fails = 0;
                stalled = false;
                continue;
            }

            // D. Handle incoming pose telemetry directly as ground truth
            if (msg.find("\"x\"") != std::string::npos && msg.find("\"yaw\"") != std::string::npos && msg.find("\"action\"") == std::string::npos) {
                double px = extract_json_val(msg, "x");
                double py = extract_json_val(msg, "y");
                double pyaw = extract_json_val(msg, "yaw");

                // Apply the lidar-vs-map correction on top of the incoming pose
                px += loc_off_x;
                py += loc_off_y;
                pyaw = wrap_pi(pyaw + loc_off_yaw);
                pose_seen = true;

                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.pose.x = px;
                    state.pose.y = py;
                    state.pose.yaw = pyaw;
                    state.gps.rtk_fixed = (msg.find("Fixed RTK") != std::string::npos);
                }

                // Synchronize pose estimator with true RTK ground truth
                pose_estimator.set_pose(px, py, pyaw);
                update_rover_pos_file("rtk/rover_pos.dat", {px, py, pyaw});
            }

            // E. Handle incoming rtk GNSS fix info for lat/lon display
            // (matched loosely so it works whether or not the JSON has spaces after the colons)
            if (msg.find("\"rtk\"") != std::string::npos && msg.find("\"lat\"") != std::string::npos) {
                double lat = extract_json_val(msg, "lat");
                double lon = extract_json_val(msg, "lon");
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.gps.latitude = lat;
                    state.gps.longitude = lon;
                }

                // RTK position in map coordinates (x east, y north from the lat/lon of map 0,0)
                const bool good_fix = (msg.find("Fixed RTK") != std::string::npos ||
                                       msg.find("Float RTK") != std::string::npos) &&
                                      !(lat == 0.0 && lon == 0.0);
                gps_good = good_fix;
                if (good_fix) {
                    gps_x = (lon - kOriginLon) * 111320.0 * std::cos(kOriginLat * M_PI / 180.0);
                    gps_y = (lat - kOriginLat) * 111320.0;
                    gps_time = now;

                    // The pose in use and the RTK position must agree; if not, the rover and the pose source
                    // are using different map origins (or the pose source has drifted).
                    if (pose_seen) {
                        Pose pp;
                        {
                            std::lock_guard<std::mutex> lock(state.state_mutex);
                            pp = state.pose;
                        }
                        double err = std::hypot(gps_x - pp.x, gps_y - pp.y);
                        if (err > 2.0 && std::chrono::duration<double>(now - last_gps_warn).count() > 10.0) {
                            last_gps_warn = now;
                            std::cerr << "[GPS] RTK says (" << gps_x << ", " << gps_y << ") but the pose in use is ("
                                      << pp.x << ", " << pp.y << "): " << err
                                      << " m apart. Different map origin between rover and visualiser?\n";
                        }
                    }
                }
            }

            // F. Handle incoming RPLidar Scan data over UDP (port 5005)
            if (msg.find("lidar") != std::string::npos && msg.find("points") != std::string::npos) {
                size_t p_pos = msg.find("\"points\"");
                if (p_pos != std::string::npos) {
                    size_t bracket_pos = msg.find('[', p_pos);
                    if (bracket_pos != std::string::npos && msg.find('[', bracket_pos + 1) != std::string::npos) {
                        Pose current_pose_local;
                        {
                            std::lock_guard<std::mutex> lock(state.state_mutex);
                            current_pose_local = state.pose;
                        }

                        if (!got_first_scan) {
                            got_first_scan = true;
                            std::cout << "[Map] First LiDAR scan received.\n";
                        }

                        // Anchor a NEW persistent map on the first scan (skipped when a saved map was loaded)
                        if (!world_map.initialised()) {
                            world_map.init(0.0, 0.0);
                            std::cout << "[Map] Started a new persistent map.\n";
                        }

                        // Container for Local Costmap scan ingestion
                        std::vector<std::pair<double, double>> local_lidar_scans;

                        size_t curr = bracket_pos;
                        while (true) {
                            size_t pair_start = msg.find('[', curr);
                            if (pair_start == std::string::npos) break;
                            size_t pair_end = msg.find(']', pair_start);
                            if (pair_end == std::string::npos) break;

                            std::string pair_str = msg.substr(pair_start + 1, pair_end - pair_start - 1);
                            size_t comma = pair_str.find(',');
                            if (comma != std::string::npos) {
                                try {
                                    double angle_deg = std::stod(pair_str.substr(0, comma));
                                    double dist_mm = std::stod(pair_str.substr(comma + 1));
                                    double dist_m = dist_mm / 1000.0;

                                    if (dist_m > 0.1 && dist_m <= 12.0) {
                                        double body_angle = -angle_deg * M_PI / 180.0;

                                        // 1. Push to Local Costmap scan vector {range_m, angle_rad}
                                        local_lidar_scans.push_back({dist_m, body_angle});
                                    }
                                } catch (...) {}
                            }
                            curr = pair_end + 1;
                        }

                        // Relocalise once after a restart with a saved map. GPS gives the position (the map frame IS
                        // defined by the lat/lon of (0,0)), so only the heading is unknown: sweep the full circle.
                        if (kLocalise && !reloc_done && !local_lidar_scans.empty()) {
                            const bool gps_ok = gps_good && std::chrono::duration<double>(now - gps_time).count() < 2.0;
                            Pose seed = current_pose_local;
                            if (gps_ok) { seed.x = gps_x; seed.y = gps_y; }

                            auto mr = world_map.matchScan(seed.x, seed.y, seed.yaw, local_lidar_scans, 0.3, M_PI);
                            ++reloc_tries;
                            const bool moved = (mr.dx != 0.0 || mr.dy != 0.0 || mr.dyaw != 0.0);

                            if (mr.ok && mr.score >= 0.4 * mr.beams &&
                                (!moved || mr.score - mr.base >= 0.1 * mr.beams)) {
                                Pose fin{seed.x + mr.dx, seed.y + mr.dy, wrap_pi(seed.yaw + mr.dyaw)};
                                // Offset relative to the raw incoming pose stream (state.pose already contains loc_off_*)
                                loc_off_x   = fin.x - (current_pose_local.x - loc_off_x);
                                loc_off_y   = fin.y - (current_pose_local.y - loc_off_y);
                                loc_off_yaw = wrap_pi(fin.yaw - (current_pose_local.yaw - loc_off_yaw));
                                loc_base_x = loc_off_x; loc_base_y = loc_off_y; loc_base_yaw = loc_off_yaw;

                                current_pose_local = fin;
                                {
                                    std::lock_guard<std::mutex> lock(state.state_mutex);
                                    state.pose.x = fin.x;
                                    state.pose.y = fin.y;
                                    state.pose.yaw = fin.yaw;
                                }
                                pose_estimator.set_pose(fin.x, fin.y, fin.yaw);
                                reloc_done = true;
                                std::cout << "[Loc] Relocalised on saved map using " << (gps_ok ? "GPS position" : "incoming pose")
                                          << ": pose (" << std::fixed << std::setprecision(2) << fin.x << ", " << fin.y
                                          << "), heading change " << mr.dyaw * 180.0 / M_PI << " deg, match "
                                          << mr.score << "/" << mr.beams << "\n";
                            } else if (reloc_tries >= 10) {
                                reloc_done = true;
                                std::cout << "[Loc] Could not relocalise against the saved map after " << reloc_tries
                                          << " scans (not enough map in view?). Continuing with the incoming pose.\n";
                            }
                        }

                        // Localise: match the scan against the map BEFORE adding it, so the map is built at the
                        // corrected pose. Skipped automatically where the map is still empty.
                        if (kLocalise && !local_lidar_scans.empty()) {
                            auto mr = world_map.matchScan(current_pose_local.x, current_pose_local.y,
                                                          current_pose_local.yaw, local_lidar_scans,
                                                          kLocWinM, kLocWinDeg * M_PI / 180.0);
                            if (!mr.ok) {
                                ++loc_rejects;
                            } else if (mr.dx != 0.0 || mr.dy != 0.0 || mr.dyaw != 0.0) {
                                double cx = kLocGain * mr.dx, cy = kLocGain * mr.dy, cyaw = kLocGain * mr.dyaw;
                                double nox = loc_off_x + cx, noy = loc_off_y + cy;
                                double noyaw = wrap_pi(loc_off_yaw + cyaw);

                                if (std::hypot(nox - loc_base_x, noy - loc_base_y) > kLocMaxOffM ||
                                    std::abs(wrap_pi(noyaw - loc_base_yaw)) > kLocMaxOffDeg * M_PI / 180.0) {
                                    std::cerr << "[Loc] Correction drifted too far from the relocalised pose; "
                                                 "resetting it. Is the pose source consistent with the map?\n";
                                    loc_off_x = loc_base_x; loc_off_y = loc_base_y; loc_off_yaw = loc_base_yaw;
                                } else {
                                    loc_off_x = nox; loc_off_y = noy; loc_off_yaw = noyaw;
                                    current_pose_local.x += cx;
                                    current_pose_local.y += cy;
                                    current_pose_local.yaw = wrap_pi(current_pose_local.yaw + cyaw);
                                    {
                                        std::lock_guard<std::mutex> lock(state.state_mutex);
                                        state.pose.x = current_pose_local.x;
                                        state.pose.y = current_pose_local.y;
                                        state.pose.yaw = current_pose_local.yaw;
                                    }
                                    pose_estimator.set_pose(current_pose_local.x, current_pose_local.y,
                                                            current_pose_local.yaw);
                                    ++loc_accepts;
                                }
                            } else {
                                ++loc_accepts; // matched with no correction needed
                            }
                        }

                        // Integrate this scan into the persistent world-anchored map (used by Theta*)
                        world_map.integrate(current_pose_local.x, current_pose_local.y,
                                            current_pose_local.yaw, local_lidar_scans);

                        // Update the local costmap (short-range safety layer)
                        local_costmap.update(local_lidar_scans, current_pose_local.x, current_pose_local.y, current_pose_local.yaw);

                        // Is the current path still clear? Checked against a slightly SMALLER inflation than the
                        // planner uses (80%) and only after several scans in a row, so it can't flicker at the edge.
                        if (has_active_goal && !is_teleop_active && global_path.size() >= 2 &&
                            world_map.pathBlocked(global_path, current_pose_local.x, current_pose_local.y,
                                                  kInflateM * 0.8, kClearM)) {
                            if (++blocked_scans >= kBlockedScans) request_replan("path blocked by new obstacle");
                        } else {
                            blocked_scans = 0;
                        }

                        // Free distance straight ahead ALONG THE ROVER'S HEADING (the old check looked along
                        // grid +y, i.e. world north, regardless of where the rover was facing).
                        // Three lanes (centre, +-0.2 m) approximate the rover's width.
                        {
                            const auto& lgrid = local_costmap.getGrid();
                            const int cw = local_costmap.getWidth();
                            const int ch = local_costmap.getHeight();
                            const double hx = std::cos(current_pose_local.yaw);
                            const double hy = std::sin(current_pose_local.yaw);

                            forward_clearance = 99.0;
                            for (double d = 0.15; d <= 1.5 && forward_clearance > 98.0; d += local_res) {
                                for (double lat : {-0.2, 0.0, 0.2}) {
                                    double ddx = d * hx - lat * hy;
                                    double ddy = d * hy + lat * hx;
                                    int cx = static_cast<int>(std::floor(ddx / local_res)) + cw / 2;
                                    int cy = static_cast<int>(std::floor(ddy / local_res)) + ch / 2;
                                    if (cx < 0 || cx >= cw || cy < 0 || cy >= ch) continue;
                                    if (lgrid[cy * cw + cx].cost >= 100) { // lethal LiDAR hit
                                        forward_clearance = d;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // --- 20 Hz CONTROL LOOP STEP ---
        if (dt > 0.0) {
            Pose current_pose;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                current_pose = state.pose;
            }

            // --- Replan triggers (only while navigating) ---
            if (has_active_goal && !is_teleop_active) {
                const double since_replan = std::chrono::duration<double>(now - last_replan_time).count();

                if (!replan_reason) {
                    if (dist_to_path(global_path, current_pose.x, current_pose.y) > kOffPathM) {
                        request_replan("rover left the path");
                    } else if (since_replan >= kBackstopS) {
                        request_replan("periodic backstop");
                    }
                }

                if (replan_reason && since_replan >= kCooldownS) {
                    std::cout << "[Planner] Replanning: " << replan_reason << "\n";
                    last_replan_time = now;
                    if (plan_and_publish(current_pose, active_goal, false)) {
                        replan_fails = 0;
                        blocked_scans = 0;
                    } else if (++replan_fails >= kMaxReplanFails) {
                        std::cerr << "[Planner] Goal unreachable after " << replan_fails
                                  << " failed replans. Cancelling goal.\n";
                        has_active_goal = false;
                        global_path.clear();
                        follower.set_path({});
                        replan_fails = 0;
                    } else {
                        std::cerr << "[Planner] Replan failed; keeping previous path.\n";
                    }
                    replan_reason = nullptr;
                }
            }

            // Control Command Arbitration (Teleop > Active Goal > Idle)
            if (is_teleop_active) {
                // Teleop active: leave manual control open
            } else if (has_active_goal) {
                auto [cmd, done] = follower.update(current_pose.x, current_pose.y, current_pose.yaw);

                if (done) {
                    std::cout << "[Planner] Destination reached!\n";
                    has_active_goal = false;
                    is_teleop_active = false;
                    global_path.clear();
                    follower.set_path({});
                    cmd.linear = 0.0;
                    cmd.angular = 0.0;
                }

                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    // Safety layer applied AFTER the follower (the old override was written to
                    // motor_command and then immediately overwritten by this loop every 50 ms).
                    // Slows down as a wall gets close and stops forward motion at 0.35 m; angular
                    // is left alone so the rover can still turn away.
                    bool stalling = false;
                    if (!done && cmd.linear > 0.0) {
                        double scale = 1.0;
                        if (forward_clearance < 0.35)      scale = 0.0;
                        else if (forward_clearance < 1.0)  scale = (forward_clearance - 0.35) / 0.65;
                        stalling = (scale == 0.0);
                        cmd.linear *= scale;
                    }
                    // Forward motion held by the safety layer for too long -> ask for a replan
                    if (stalling) {
                        if (!stalled) {
                            stalled = true;
                            stall_since = now;
                        } else if (std::chrono::duration<double>(now - stall_since).count() > kStallS) {
                            request_replan("stalled by obstacle ahead");
                            stall_since = now;
                        }
                    } else {
                        stalled = false;
                    }
                    state.velocity = cmd;
                    state.motor_command.linear = cmd.linear;
                    state.motor_command.angular = cmd.angular;
                    state.motor_command.enabled = true;
                }

                // Broadcast cmd_vel over UGV protocol on port 5006
                std::ostringstream cmd_json;
                cmd_json << "{\"type\":\"cmd_vel\",\"v\":1,\"ts\":" << get_timestamp() 
                         << ",\"linear\":" << cmd.linear 
                         << ",\"angular\":" << cmd.angular << "}";
                udp_bus.send(5006, cmd_json.str());
            } else {
                // Idle state: safely disable motor commands without terminating app
                std::lock_guard<std::mutex> lock(state.state_mutex);
                state.velocity = {0.0, 0.0};
                state.motor_command.linear = 0.0;
                state.motor_command.angular = 0.0;
                state.motor_command.enabled = false;
            }
        }

        // --- AUTOSAVE the global map (only when it has changed) ---
        if (world_map.initialised() &&
            std::chrono::duration<double>(now - last_save_time).count() >= kSaveEveryS) {
            last_save_time = now;
            if (world_map.consumeDirty()) {
                if (world_map.save(kMapFile, kPreviewFile)) {
                    std::cout << "[Map] Saved " << kMapFile << "\n";
                }
            }
        }

        // --- TELEMETRY OUTPUT (Every 1 Second) ---
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() >= 1000) {
            Pose p;
            Velocity v;
            double lat = 0, lon = 0;
            bool rtk = false;

            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                p = state.pose;
                v = state.velocity;
                lat = state.gps.latitude;
                lon = state.gps.longitude;
                rtk = state.gps.rtk_fixed;
            }

            std::cout << "[NAV] Pose: (" << std::fixed << std::setprecision(2) << p.x << ", " << p.y << ") m "
                      << "| Yaw: " << std::setprecision(2) << p.yaw << " rad "
                      << "| Cmd (v/w): " << v.linear << " m/s / " << v.angular << " rad/s "
                      << "| Mode: " << (is_teleop_active ? "TELEOP" : (has_active_goal ? "NAVIGATING" : "IDLE")) << " "
                      << "| GPS: " << std::setprecision(5) << lat << "," << lon << " [" << (rtk ? "RTK" : "FLOAT") << "]"
                      << " | Loc off: (" << std::setprecision(2) << loc_off_x << "," << loc_off_y << ","
                      << loc_off_yaw * 180.0 / M_PI << "deg) ok/rej " << loc_accepts << "/" << loc_rejects
                      << std::endl;

            last_log_time = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 Hz
    }

    std::cout << "\n[Main] Shutting down...\n";
    if (world_map.initialised()) {
        if (world_map.save(kMapFile, kPreviewFile)) std::cout << "[Map] Saved " << kMapFile << "\n";
    }

    motor_driver.stop();
    debug_server.stop();

    return 0;
}