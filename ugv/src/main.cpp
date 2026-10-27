#include "rover/planning/lcostmap.hpp"
#include "rover/state/rover_state.hpp"
#include "rover/gps/gps_driver.hpp"
#include "rover/motors/motor_interface.hpp"
#include "rover/debug/debug_server.hpp"
#include "rover/localization/pose_estimator.hpp"
#include "rover/planning/theta_star.hpp"
#include "rover/planning/follower.hpp"
#include "rover/comms/udp_bus.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

// ============================================================================
// Tuning
// ============================================================================
constexpr int    kMapW        = 600;
constexpr int    kMapH        = 600;
constexpr double kMapRes      = 0.10;   // m per planning/map cell
constexpr double kInflateM    = 0.40;   // keep this far from walls (same as the visualiser's --inflate default)
constexpr double kClearM      = 0.30;   // inflated cells this close to the rover are treated as free
constexpr double kFootprintM  = 0.25;   // cells this close to the rover can never be walls
constexpr double kLocalRes    = 0.05;   // local (forward clearance) costmap resolution

constexpr double kLidarMinM   = 0.15;
constexpr double kLidarMaxM   = 10.0;   // beyond this a small pose error moves a wall too far
constexpr double kFreeRayM    = 5.0;    // beams with no return are treated as free this far
constexpr double kEraseRangeM = 5.0;    // walls are only ever ERASED within this range of the lidar. Farther away they persist,
                                        // and so do walls you have driven away from (nothing sees through them)
constexpr bool   kInferFreeFromMissing = true;   // simulated lidar drops beams that hit nothing; set false for real hardware
constexpr double kMaxBiasRad = 15.0 * M_PI / 180.0;   // largest heading correction the map matcher may hold
constexpr double kMapMaxYawRate = 0.3;  // rad/s: do not add scans to the map while turning faster than this
constexpr double kTurnSlowW     = 0.3;  // above this commanded turn rate the rover slows down (same arc, driven slower)
constexpr double kTurnSlowMin   = 0.5;  // ...down to this fraction of speed at full turn rate

constexpr double kPoseTimeoutS = 0.5;   // hold still if no pose message for this long
constexpr double kScanTimeoutS = 1.5;   // hold still if no lidar scan for this long
constexpr bool   kRequireFreshLidar = true;

constexpr double kReplanPeriodS = 0.5;  // how often the path is CHECKED. It is only replaced if blocked or left
constexpr double kOffPathM      = 1.0;  // also replan if the rover ends up this far from its path
constexpr double kWaypointTolM  = 0.4;  // a route waypoint counts as reached when the rover is this close

using Clock = std::chrono::steady_clock;
using Ray   = std::pair<double, double>;   // (range m, body-frame angle rad)

// ============================================================================
// Process-level safety nets
// ============================================================================
std::atomic<bool>        g_running{true};
std::atomic<const char*> g_mode{"STARTING"};

const char* signal_name(int sig) {
    switch (sig) {
        case SIGSEGV: return "SIGSEGV (segmentation fault)";
        case SIGABRT: return "SIGABRT (abort / terminate)";
        case SIGFPE:  return "SIGFPE (arithmetic error)";
        default:      return "unknown signal";
    }
}

void crash_handler(int sig) {
    std::cerr << "\n=========================================\n"
              << "[FATAL] Signal " << sig << ": " << signal_name(sig) << "\n"
              << "Mode when it happened: " << g_mode.load() << "\n"
              << "=========================================\n"
              << "Press ENTER to close..." << std::endl;
    std::string dummy;
    std::getline(std::cin, dummy);
    std::_Exit(sig);
}

void terminate_handler() {
    std::cerr << "\n[TERMINATE] Uncaught exception (mode: " << g_mode.load() << ")";
    try {
        if (auto ep = std::current_exception()) std::rethrow_exception(ep);
    } catch (const std::exception& e) {
        std::cerr << ": " << e.what();
    } catch (...) {
        std::cerr << ": unknown exception";
    }
    std::cerr << "\nPress ENTER to close..." << std::endl;
    std::string dummy;
    std::getline(std::cin, dummy);
    std::_Exit(3);
}

void on_sigint(int) { g_running = false; }

// ============================================================================
// Small helpers
// ============================================================================
double wrap_angle(double a) {
    return a - 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
}

double now_wall() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

double secs_between(Clock::time_point later, Clock::time_point earlier) {
    return std::chrono::duration<double>(later - earlier).count();
}

// ---- tiny JSON field readers (the bus messages are flat, single-line JSON) ----
bool json_number(const std::string& s, const char* key, double& out) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return false;
    p = s.find(':', p + k.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < s.size() && (s[p] == ' ' || s[p] == '\t' || s[p] == '"')) ++p;
    if (p >= s.size()) return false;
    const char* b = s.c_str() + p;
    char* e = nullptr;
    const double v = std::strtod(b, &e);
    if (e == b) return false;
    out = v;
    return true;
}

std::string json_string(const std::string& s, const char* key) {
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return "";
    p = s.find(':', p + k.size());
    if (p == std::string::npos) return "";
    ++p;
    while (p < s.size() && (s[p] == ' ' || s[p] == '\t')) ++p;
    if (p >= s.size() || s[p] != '"') return "";
    ++p;
    const size_t e = s.find('"', p);
    if (e == std::string::npos) return "";
    return s.substr(p, e - p);
}

// "key":[[x,y],[x,y],...]  ->  out.  False if the key is missing or the list is empty/malformed.
bool json_xy_list(const std::string& s, const char* key, std::vector<Pose>& out) {
    out.clear();
    const std::string k = std::string("\"") + key + "\"";
    size_t p = s.find(k);
    if (p == std::string::npos) return false;
    p = s.find('[', p + k.size());
    if (p == std::string::npos) return false;
    size_t cur = p + 1;
    while (true) {
        const size_t nb = s.find('[', cur);
        const size_t ne = s.find(']', cur);
        if (ne == std::string::npos) break;
        if (nb == std::string::npos || ne < nb) break;   // the closing bracket of the whole list
        const char* b = s.c_str() + nb + 1;
        char* e1 = nullptr;
        const double x = std::strtod(b, &e1);
        if (e1 == b) { out.clear(); return false; }
        while (*e1 == ' ' || *e1 == ',') ++e1;
        char* e2 = nullptr;
        const double y = std::strtod(e1, &e2);
        if (e2 == e1) { out.clear(); return false; }
        out.push_back(Pose{x, y, 0.0});
        cur = ne + 1;
    }
    return !out.empty();
}

enum class MsgKind { Other, Goal, Pose, Lidar };

MsgKind classify(const std::string& m, std::string& action) {
    const std::string type = json_string(m, "type");
    action = json_string(m, "action");
    if (type == "goal")  return MsgKind::Goal;
    if (type == "pose")  return MsgKind::Pose;
    if (type == "lidar") return MsgKind::Lidar;
    if (type == "cmd_vel" || type == "path" || type == "rtk" || type == "esp") return MsgKind::Other;
    // No usable "type": fall back to what the message looks like
    if (!action.empty()) return MsgKind::Goal;
    if (m.find("\"points\"") != std::string::npos && m.find("lidar") != std::string::npos) return MsgKind::Lidar;
    if (m.find("\"x\"") != std::string::npos && m.find("\"yaw\"") != std::string::npos) return MsgKind::Pose;
    return MsgKind::Other;
}

// ---- outgoing messages ----
std::string cmd_vel_json(double lin, double ang) {
    std::ostringstream s;
    s << "{\"type\":\"cmd_vel\",\"v\":1,\"ts\":" << std::fixed << std::setprecision(3) << now_wall()
      << std::setprecision(4) << ",\"linear\":" << lin << ",\"angular\":" << ang << "}";
    return s.str();
}

std::string path_json(const std::vector<Pose>& path) {
    std::ostringstream s;
    s << "{\"type\":\"path\",\"v\":1,\"ts\":" << std::fixed << std::setprecision(3) << now_wall() << ",\"points\":[";
    for (size_t i = 0; i < path.size(); ++i) {
        s << "[" << path[i].x << "," << path[i].y << "]";
        if (i + 1 < path.size()) s << ",";
    }
    s << "]}";
    return s.str();
}

// rtk/rover_pos.dat: live JSON pose telemetry (unchanged from the original: constant lat/lon, live enu_x/enu_y/yaw)
void write_rover_pos_file(const std::string& filepath, const Pose& pose) {
    std::ofstream out(filepath, std::ios::trunc);
    if (!out.is_open()) return;
    out << std::fixed << std::setprecision(9)
        << "{\"lat\": -31.950500067"
        << ", \"lon\": 115.860489505"
        << ", \"fix\": \"Fixed RTK\""
        << ", \"sats\": 23"
        << ", \"hdop\": 0.67"
        << ", \"corr_age\": 0.8"
        << ", \"ts\": " << std::setprecision(7) << now_wall()
        << ", \"enu_x\": " << std::setprecision(2) << pose.x
        << ", \"enu_y\": " << pose.y
        << ", \"yaw\": " << std::setprecision(4) << pose.yaw
        << "}\n";
}

// ============================================================================
// Pose history: lets a lidar scan be placed at the pose the rover had when the
// scan was taken, not at whatever pose arrived last.
// ============================================================================
class PoseHistory {
public:
    struct Sample { double t, x, y, yaw; };

    void add(double t, double x, double y, double yaw) {
        if (!buf_.empty()) {
            if (t < buf_.back().t - 1.0) buf_.clear();   // sender restarted or clock jumped
            else if (t <= buf_.back().t) return;         // duplicate / out of order
        }
        buf_.push_back({t, x, y, yaw});
        while (buf_.size() > 128 || (buf_.back().t - buf_.front().t) > 3.0) buf_.pop_front();
    }

    // Pose at time t (interpolated, or extrapolated by at most 0.25 s). yaw_rate in rad/s.
    bool lookup(double t, Sample& out, double& yaw_rate) const {
        const size_t n = buf_.size();
        if (n < 2) return false;
        if (t < buf_.front().t - 0.05) return false;
        size_t i;
        if (t >= buf_.back().t) {
            if (t - buf_.back().t > kMaxExtrapolateS) return false;
            i = n - 2;
        } else {
            i = 0;
            while (i + 2 < n && buf_[i + 1].t < t) ++i;
        }
        const Sample& a = buf_[i];
        const Sample& b = buf_[i + 1];
        const double span = b.t - a.t;
        if (span <= 0.0 || span > 0.5) return false;   // gap in the pose stream: do not guess
        const double f = (t - a.t) / span;
        const double dyaw = wrap_angle(b.yaw - a.yaw);
        out = {t, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), wrap_angle(a.yaw + f * dyaw)};
        yaw_rate = dyaw / span;
        return true;
    }

private:
    static constexpr double kMaxExtrapolateS = 0.25;
    std::deque<Sample> buf_;
};

// ============================================================================
// Occupancy map
//  - walls need several hits to appear and can be erased again by beams that pass through them
//  - re-centering keeps what the rover has learned instead of wiping the map
// ============================================================================
class OccupancyMap {
public:
    OccupancyMap(int w, int h, double res)
        : w_(w), h_(h), res_(res), lo_(static_cast<size_t>(w) * h, 0), flags_(static_cast<size_t>(w) * h, 0) {}

    bool   initialised() const { return init_; }
    double originX() const { return ox_; }
    double originY() const { return oy_; }

    void recenterIfNeeded(double rx, double ry) {
        if (!init_) {
            ox_ = rx - (w_ * res_) / 2.0;
            oy_ = ry - (h_ * res_) / 2.0;
            init_ = true;
            return;
        }
        const double margin = 8.0;
        if (rx > ox_ + margin && rx < ox_ + w_ * res_ - margin &&
            ry > oy_ + margin && ry < oy_ + h_ * res_ - margin) return;

        const double new_ox = rx - (w_ * res_) / 2.0;
        const double new_oy = ry - (h_ * res_) / 2.0;
        const int shx = static_cast<int>(std::lround((new_ox - ox_) / res_));
        const int shy = static_cast<int>(std::lround((new_oy - oy_) / res_));
        std::cout << "[OccupancyMap] Re-centering map around (" << rx << ", " << ry << "), keeping known cells\n";

        std::vector<int8_t> moved(lo_.size(), 0);
        for (int y = 0; y < h_; ++y) {
            const int sy = y + shy;
            if (sy < 0 || sy >= h_) continue;
            for (int x = 0; x < w_; ++x) {
                const int sx = x + shx;
                if (sx < 0 || sx >= w_) continue;
                moved[y * w_ + x] = lo_[sy * w_ + sx];
            }
        }
        lo_.swap(moved);
        ox_ += shx * res_;
        oy_ += shy * res_;
    }

    bool toCell(double wx, double wy, int& cx, int& cy) const {
        cx = static_cast<int>(std::floor((wx - ox_) / res_));
        cy = static_cast<int>(std::floor((wy - oy_) / res_));
        return inBounds(cx, cy);
    }

    // One scan updates each cell at most once:
    //   a cell a beam ENDED in is a hit (+5)
    //   else a cell a beam PASSED THROUGH, within kEraseRangeM, is seen-free (-1)
    //   cells next to a hit are never decremented (grazing beams and range noise would otherwise eat real walls)
    // Beyond kEraseRangeM nothing is erased, and cells the lidar can't see are never touched, so a wall stays
    // in the map after you drive away from it.
    void integrate(double rx, double ry, double ryaw, const std::vector<Ray>& hits, const std::vector<Ray>& free_rays) {
        recenterIfNeeded(rx, ry);
        int sx, sy;
        if (!toCell(rx, ry, sx, sy)) return;
        touched_.clear();

        protectSurfaces(rx, ry, ryaw, hits);
        for (const auto& s : free_rays) {
            const double hx = rx + s.first * std::cos(s.second + ryaw);
            const double hy = ry + s.first * std::sin(s.second + ryaw);
            walk(sx, sy, cellX(hx), cellY(hy), false);
        }
        for (const auto& s : hits) {
            const double hx = rx + s.first * std::cos(s.second + ryaw);
            const double hy = ry + s.first * std::sin(s.second + ryaw);
            walk(sx, sy, cellX(hx), cellY(hy), true);
        }

        for (int idx : touched_) {
            const uint8_t f = flags_[idx];
            if (f & kFlagHit)                                  bump(idx, kHit);
            else if ((f & kFlagPassed) && !(f & kFlagProtect)) bump(idx, kMiss);
            flags_[idx] = 0;
        }
    }

    // Heading correction by scan matching. Position (RTK) is trusted; only the heading is searched, within
    // +-kMatchMaxDeg of `centre_yaw`. Returns true and the best total yaw when the match is reliable.
    // Call BEFORE integrate(), so the scan is compared with what was learned earlier.
    bool matchYaw(double rx, double ry, double centre_yaw, const std::vector<Ray>& hits, double& best_yaw, double& fit) const {
        std::vector<Ray> use;
        for (const auto& h : hits) if (h.first > 0.3 && h.first < kMatchRangeM) use.push_back(h);
        if (use.size() < 25 || occupiedCount() < 60) return false;
        constexpr int kN = 25;                       // -6..+6 deg in 0.5 deg steps
        constexpr double kStep = 0.5 * M_PI / 180.0;
        double score[kN];
        const double s2 = 2.0 * 0.7 * 0.7;           // blur sigma 0.7 cells
        for (int i = 0; i < kN; ++i) {
            const double yaw = centre_yaw + (i - kN / 2) * kStep;
            double sum = 0.0;
            for (const auto& h : use) {
                const double fx = (rx + h.first * std::cos(h.second + yaw) - ox_) / res_;
                const double fy = (ry + h.first * std::sin(h.second + yaw) - oy_) / res_;
                const int cx = static_cast<int>(std::floor(fx)), cy = static_cast<int>(std::floor(fy));
                double best = 0.0;
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx) {
                        const int nx = cx + dx, ny = cy + dy;
                        if (!inBounds(nx, ny) || lo_[ny * w_ + nx] < kOccThresh) continue;
                        const double ex = fx - (nx + 0.5), ey = fy - (ny + 0.5);
                        best = std::max(best, std::exp(-(ex * ex + ey * ey) / s2));
                    }
                sum += best;
            }
            score[i] = sum / use.size() - 0.002 * std::abs(i - kN / 2);   // ties go to the smaller correction
        }
        int bi = 0;
        for (int i = 1; i < kN; ++i) if (score[i] > score[bi]) bi = i;
        if (bi == 0 || bi == kN - 1) return false;                         // best answer on the edge of the search: not trustworthy
        if (score[bi] < 0.35 || score[bi] - score[kN / 2] < 0.01) return false;
        const double denom = score[bi - 1] - 2 * score[bi] + score[bi + 1];
        const double off = std::abs(denom) < 1e-9 ? 0.0 : std::clamp(0.5 * (score[bi - 1] - score[bi + 1]) / denom, -1.0, 1.0);
        best_yaw = centre_yaw + (bi - kN / 2 + off) * kStep;
        fit = score[bi];
        return true;
    }

    void reset() { std::fill(lo_.begin(), lo_.end(), static_cast<int8_t>(0)); }

    // The rover is physically here, so nothing within radius_m of it can be a wall.
    void clearFootprint(double rx, double ry, double radius_m) {
        int cx, cy;
        if (!toCell(rx, ry, cx, cy)) return;
        const int r = static_cast<int>(std::ceil(radius_m / res_));
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (dx * dx + dy * dy > r * r) continue;
                const int nx = cx + dx, ny = cy + dy;
                if (!inBounds(nx, ny)) continue;
                int8_t& v = lo_[ny * w_ + nx];
                if (v > 0) v = 0;
            }
        }
    }

    // 0 = free, 1 = inflated, 2 = wall
    std::vector<uint8_t> buildPlanningGrid(double rx, double ry, int inflate_cells, int clear_cells) const {
        std::vector<uint8_t> g(static_cast<size_t>(w_) * h_, 0);
        const int r2 = inflate_cells * inflate_cells;

        for (int y = 0; y < h_; ++y) {
            for (int x = 0; x < w_; ++x) {
                if (lo_[y * w_ + x] < kOccThresh) continue;
                for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
                    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
                        if (dx * dx + dy * dy > r2) continue;
                        const int nx = x + dx, ny = y + dy;
                        if (!inBounds(nx, ny)) continue;
                        uint8_t& c = g[ny * w_ + nx];
                        if (c == 0) c = 1;
                    }
                }
            }
        }
        for (size_t i = 0; i < g.size(); ++i) {
            if (lo_[i] >= kOccThresh) g[i] = 2;
        }

        int rcx, rcy;
        if (toCell(rx, ry, rcx, rcy)) {
            const int c2 = clear_cells * clear_cells;
            for (int dy = -clear_cells; dy <= clear_cells; ++dy) {
                for (int dx = -clear_cells; dx <= clear_cells; ++dx) {
                    if (dx * dx + dy * dy > c2) continue;
                    const int nx = rcx + dx, ny = rcy + dy;
                    if (inBounds(nx, ny) && g[ny * w_ + nx] == 1) g[ny * w_ + nx] = 0;
                }
            }
        }
        return g;
    }

    // Would the planner's straight-line check reject the segment A->B on this grid? Walks the same cells the
    // planner walks. Cells before position u_start along the segment (already driven past) are ignored.
    bool segmentBlocked(const std::vector<uint8_t>& g, double ax, double ay, double bx, double by, double u_start) const {
        const int x0 = cellX(ax), y0 = cellY(ay), x1 = cellX(bx), y1 = cellY(by);
        const double dxw = bx - ax, dyw = by - ay;
        const double l2 = dxw * dxw + dyw * dyw;
        const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        const int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (true) {
            if (inBounds(x, y)) {
                double u = 1.0;
                if (l2 > 0.0) u = ((ox_ + (x + 0.5) * res_ - ax) * dxw + (oy_ + (y + 0.5) * res_ - ay) * dyw) / l2;
                if (u >= u_start - 0.03 && g[y * w_ + x] != 0) return true;
            }
            if (x == x1 && y == y1) break;
            const int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }
        return false;
    }

    int occupiedCount() const {
        int n = 0;
        for (int8_t v : lo_) if (v >= kOccThresh) ++n;
        return n;
    }

private:
    static constexpr int kOccThresh = 12;   // 3 hits to become a wall
    static constexpr int kHit  = +5;
    static constexpr int kMiss = -1;        // a ghost wall is gone after ~9 clean passes
    static constexpr int kMaxL = 30;        // a well-seen wall needs ~19 clean passes to disappear
    static constexpr int kMinL = -15;

    bool inBounds(int x, int y) const { return x >= 0 && x < w_ && y >= 0 && y < h_; }
    int  cellX(double wx) const { return static_cast<int>(std::floor((wx - ox_) / res_)); }
    int  cellY(double wy) const { return static_cast<int>(std::floor((wy - oy_) / res_)); }

    static constexpr uint8_t kFlagHit = 1, kFlagPassed = 2, kFlagProtect = 4;
    static constexpr int kProtectCells = 1;   // cells around a hit that are not eroded in the same scan (0 makes the wall count flicker)

    static constexpr double kMatchRangeM = 9.0;

    // Cells that lie ON a surface this scan saw: every hit, and the straight line between each pair of
    // neighbouring hits that look like the same surface. On a wall seen at a steep angle the hits are far apart;
    // the beams between them graze wall cells no beam ended in, and those cells must not be cleared.
    void protectSurfaces(double rx, double ry, double ryaw, const std::vector<Ray>& hits) {
        if (hits.size() < 2) return;
        std::vector<Ray> h(hits);
        std::sort(h.begin(), h.end(), [](const Ray& a, const Ray& b) { return a.second < b.second; });
        auto pt = [&](const Ray& r, double& x, double& y) {
            x = (rx + r.first * std::cos(r.second + ryaw) - ox_) / res_;
            y = (ry + r.first * std::sin(r.second + ryaw) - oy_) / res_;
        };
        const double join_rad = 4.0 * M_PI / 180.0;
        for (size_t i = 0; i < h.size(); ++i) {
            const Ray& a = h[i];
            const Ray& b = h[(i + 1) % h.size()];
            double gap = b.second - a.second;
            if (i + 1 == h.size()) gap += 2.0 * M_PI;
            if (gap > join_rad || std::abs(a.first - b.first) > 1.0) continue;
            double ax, ay, bx, by;
            pt(a, ax, ay); pt(b, bx, by);
            const double len = std::hypot(bx - ax, by - ay);
            if (len * res_ > 1.0) continue;
            const int n = std::max(1, static_cast<int>(std::ceil(len * 2.0)));
            for (int k = 0; k <= n; ++k) {
                const int cx = static_cast<int>(std::floor(ax + (bx - ax) * k / n));
                const int cy = static_cast<int>(std::floor(ay + (by - ay) * k / n));
                for (int oy = -1; oy <= 1; ++oy)
                    for (int ox = -1; ox <= 1; ++ox)
                        if (inBounds(cx + ox, cy + oy)) touch((cy + oy) * w_ + (cx + ox), kFlagProtect);
            }
        }
    }

    void bump(int idx, int d) {
        int8_t& v = lo_[idx];
        v = static_cast<int8_t>(std::clamp(static_cast<int>(v) + d, kMinL, kMaxL));
    }

    void touch(int idx, uint8_t flag) {
        if (flags_[idx] == 0) touched_.push_back(idx);
        flags_[idx] |= flag;
    }

    // Walk from the lidar cell (x0,y0) to the beam end (x1,y1).
    void walk(int x0, int y0, int x1, int y1, bool is_hit) {
        const double max_cells = kEraseRangeM / res_;
        const double max2 = max_cells * max_cells;
        const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        const int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (!(x == x1 && y == y1)) {
            if (inBounds(x, y)) {
                const double ddx = x - x0, ddy = y - y0;
                if (ddx * ddx + ddy * ddy <= max2) touch(y * w_ + x, kFlagPassed);
            }
            const int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 < dx)  { err += dx; y += sy; }
        }
        if (is_hit && inBounds(x1, y1)) {
            touch(y1 * w_ + x1, kFlagHit);
            for (int oy = -kProtectCells; oy <= kProtectCells; ++oy)
                for (int ox = -kProtectCells; ox <= kProtectCells; ++ox)
                    if (inBounds(x1 + ox, y1 + oy)) touch((y1 + oy) * w_ + (x1 + ox), kFlagProtect);
        }
    }

    int w_, h_;
    double res_;
    double ox_{0.0}, oy_{0.0};
    bool init_{false};
    std::vector<int8_t> lo_;
    std::vector<uint8_t> flags_;   // per-scan scratch (all zero between scans)
    std::vector<int> touched_;
};

// ============================================================================
// Path helpers. The path is only replaced when it is blocked or the rover has left it.
// ============================================================================
struct PathProgress {
    size_t seg{0};     // segment the rover is on
    double t{0.0};     // how far along that segment (0..1)
    double off{0.0};   // distance from the rover to the path (m)
};

PathProgress pathProgress(const std::vector<Pose>& p, double x, double y) {
    PathProgress pr;
    if (p.size() < 2) return pr;
    pr.off = 1e18;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        const double dx = p[i + 1].x - p[i].x, dy = p[i + 1].y - p[i].y;
        const double l2 = dx * dx + dy * dy;
        const double tt = (l2 > 0.0) ? std::clamp(((x - p[i].x) * dx + (y - p[i].y) * dy) / l2, 0.0, 1.0) : 0.0;
        const double d = std::hypot(x - (p[i].x + tt * dx), y - (p[i].y + tt * dy));
        if (d < pr.off) { pr.off = d; pr.seg = i; pr.t = tt; }
    }
    return pr;
}

bool pathBlocked(const std::vector<Pose>& p, const PathProgress& pr, const OccupancyMap& map, const std::vector<uint8_t>& grid) {
    for (size_t i = pr.seg; i + 1 < p.size(); ++i) {
        if (map.segmentBlocked(grid, p[i].x, p[i].y, p[i + 1].x, p[i + 1].y, i == pr.seg ? pr.t : 0.0)) return true;
    }
    return false;
}

// Distance to the first lidar hit in front of the rover (world-aligned local costmap).
double computeForwardClearance(LocalCostmap& cm, double yaw) {
    const auto& grid = cm.getGrid();
    const int cw = cm.getWidth();
    const int ch = cm.getHeight();
    const double hx = std::cos(yaw), hy = std::sin(yaw);
    for (double d = 0.15; d <= 1.5; d += kLocalRes) {
        for (double lat : {-0.2, 0.0, 0.2}) {
            const double ddx = d * hx - lat * hy;
            const double ddy = d * hy + lat * hx;
            const int cx = static_cast<int>(std::floor(ddx / kLocalRes)) + cw / 2;
            const int cy = static_cast<int>(std::floor(ddy / kLocalRes)) + ch / 2;
            if (cx < 0 || cx >= cw || cy < 0 || cy >= ch) continue;
            if (grid[cy * cw + cx].cost >= 100) return d;
        }
    }
    return 99.0;
}

bool due(Clock::time_point& last, Clock::time_point now, double period_s) {
    if (secs_between(now, last) >= period_s) { last = now; return true; }
    return false;
}

}  // namespace

// ============================================================================
int main() {
    std::signal(SIGSEGV, crash_handler);
    std::signal(SIGABRT, crash_handler);
    std::signal(SIGFPE,  crash_handler);
    std::signal(SIGINT,  on_sigint);
    std::set_terminate(terminate_handler);
    std::cout << std::unitbuf;   // never lose the last lines on an abnormal exit

    std::cout << "===========================================\n"
              << "    Rover Navigation System (Robust Core)  \n"
              << "===========================================\n";

    RoverState state;

    MotorInterface motor_driver(state, "COM4", 115200);
    motor_driver.start();

    DebugServer debug_server(state, 5000);
    debug_server.start();

    rover::comms::UdpBus udp_bus("239.255.42.42", "127.0.0.1");
    rover::localization::PoseEstimator pose_estimator;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
        Pose start_pose;
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.gps.rtk_fixed = false;
        start_pose = state.pose;
        pose_estimator.set_pose(start_pose.x, start_pose.y, start_pose.yaw);
    }

    // ---------------------------------------------------------------- state
    rover::planning::PurePursuit follower(0.8, 1.0, 0.6, 0.25);
    OccupancyMap world_map(kMapW, kMapH, kMapRes);
    LocalCostmap local_costmap(200, 200, kLocalRes);
    rover::planning::ThetaStarPlanner planner(kMapW, kMapH, kMapRes);
    PoseHistory history;
    double heading_bias = 0.0, logged_bias = 0.0;   // lidar-derived heading correction used for mapping only

    std::vector<Pose> global_path;
    std::vector<Pose> route;                 // waypoints still to visit; the last one is the final goal
    double inflate_m = kInflateM;            // keep-out distance for the current goal (a goal message can set it)
    Pose active_goal{0.0, 0.0, 0.0};
    bool is_teleop_active = false;
    bool has_active_goal  = false;
    bool have_pose        = false;
    bool have_scan        = false;
    double forward_clearance = 99.0;
    int zero_repeats = 0;

    auto last_pose_rx    = Clock::time_point{};
    auto last_scan_rx    = Clock::time_point{};
    auto last_replan     = Clock::now();
    auto last_log        = Clock::now();
    auto warn_pose = Clock::time_point{}, warn_lidar = Clock::time_point{}, warn_plan = Clock::time_point{};

    auto get_pose = [&]() {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        return state.pose;
    };

    auto send_cmd = [&](double lin, double ang) { udp_bus.send(5006, cmd_vel_json(lin, ang)); };

    auto stop_motors = [&](bool enabled) {
        std::lock_guard<std::mutex> lock(state.state_mutex);
        state.velocity = {0.0, 0.0};
        state.motor_command.linear = 0.0;
        state.motor_command.angular = 0.0;
        state.motor_command.enabled = enabled;
    };

    auto clear_path = [&]() {
        route.clear();
        global_path.clear();
        follower.set_path({});
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.global_path.clear();
        }
        udp_bus.send(5008, path_json({}));   // so the visualiser stops drawing the old path
    };

    // Plans from `from` through every waypoint left in `route`, as ONE continuous path (so the rover does not
    // stop and re-aim at each waypoint). Returns false if any leg has no path.
    auto plan_route = [&](const Pose& from, const std::vector<uint8_t>& grid, std::vector<Pose>& out) -> bool {
        out.clear();
        Pose start = from;
        for (size_t i = 0; i < route.size(); ++i) {
            auto leg = planner.plan_path(start, route[i], grid);
            if (leg.empty()) {
                std::cerr << "[Planner ERROR] No path from (" << start.x << ", " << start.y << ") to "
                          << (route.size() > 1 ? "waypoint " + std::to_string(i + 1) + " " : std::string("goal "))
                          << "(" << route[i].x << ", " << route[i].y << ").\n";
                out.clear();
                return false;
            }
            if (out.empty()) out = leg;
            else out.insert(out.end(), leg.begin() + 1, leg.end());
            start = leg.back();
        }
        return !out.empty();
    };

    // force = true for a brand new goal or route. Otherwise the current path is kept unless it is blocked
    // (a wall now sits on it) or the rover has ended up far from it. Plans through whatever is left in `route`.
    auto plan_and_publish = [&](const Pose& from, bool force, bool log) -> bool {
        if (route.empty()) return false;
        world_map.recenterIfNeeded(from.x, from.y);
        planner.set_origin(world_map.originX(), world_map.originY());
        world_map.clearFootprint(from.x, from.y, kFootprintM);

        const auto plan_grid = world_map.buildPlanningGrid(
            from.x, from.y,
            static_cast<int>(std::round(inflate_m / kMapRes)),
            static_cast<int>(std::round(kClearM / kMapRes)));

        if (!force && !global_path.empty()) {
            const PathProgress pr = pathProgress(global_path, from.x, from.y);
            const bool blocked  = pathBlocked(global_path, pr, world_map, plan_grid);
            const bool off_path = pr.off > kOffPathM;
            if (!blocked && !off_path) return true;   // current path is still good: keep it
            std::cout << "[Planner] Replanning: " << (blocked ? "path is blocked" : "rover is off the path") << "\n";
        }

        std::vector<Pose> path;
        if (!plan_route(from, plan_grid, path)) return false;

        global_path = path;
        follower.set_path(global_path);
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.global_path = global_path;
        }
        udp_bus.send(5008, path_json(global_path));
        if (log || !force) std::cout << "[Planner] Path published. Waypoints: " << global_path.size() << "\n";
        return true;
    };

    // ---------------------------------------------------------------- message handlers
    // Pose "snap" detector. The pose on the bus is odometry that gets corrected when telemetry arrives
    // (about once a second), so it can jump. Every jump leaves a copy of each wall seen before it in the
    // wrong place. This only measures and reports the jumps, so you can see how big they really are.
    struct PoseJump { bool have{false}; double ts{0}, x{0}, y{0}, yaw{0}; int count{0}; double worst{0}; };
    PoseJump pj;
    auto warn_jump = Clock::time_point{};

    auto handle_pose = [&](const std::string& msg, Clock::time_point now) {
        double x, y, yaw;
        if (!json_number(msg, "x", x) || !json_number(msg, "y", y) || !json_number(msg, "yaw", yaw)) return;
        double ts;
        if (!json_number(msg, "ts", ts) || ts <= 0.0) ts = now_wall();

        if (pj.have && ts > pj.ts && ts - pj.ts < 1.0) {
            const double dt_m = ts - pj.ts;
            const double extra = std::hypot(x - pj.x, y - pj.y) - 1.0 * dt_m;   // minus the fastest the rover can move
            const double dyaw = std::abs(wrap_angle(yaw - pj.yaw)) - 1.2 * dt_m;
            if (extra > 0.10 || dyaw > 0.10) {
                ++pj.count;
                pj.worst = std::max(pj.worst, std::max(extra, 0.0));
                if (due(warn_jump, now, 1.0)) {
                    std::cerr << "[Pose] JUMP: " << std::fixed << std::setprecision(2) << std::max(extra, 0.0) << " m / "
                              << std::max(dyaw, 0.0) << " rad in " << std::setprecision(2) << dt_m
                              << " s (" << pj.count << " jumps so far, worst " << pj.worst << " m)\n";
                }
            }
        }
        pj.have = true; pj.ts = ts; pj.x = x; pj.y = y; pj.yaw = yaw;

        history.add(ts, x, y, yaw);
        {
            std::lock_guard<std::mutex> lock(state.state_mutex);
            state.pose.x = x;
            state.pose.y = y;
            state.pose.yaw = yaw;
            state.gps.rtk_fixed = (msg.find("Fixed RTK") != std::string::npos);
        }
        pose_estimator.set_pose(x, y, yaw);
        have_pose = true;
        last_pose_rx = now;

        write_rover_pos_file("rtk/rover_pos.dat", Pose{x, y, yaw});
    };

    // A new goal or route: plan it and start driving. Returns false (and stops) if there is no path.
    auto start_route = [&](const std::string& msg, std::vector<Pose> wps, Clock::time_point now) {
        if (!have_pose) {
            std::cerr << "[Planner] Ignoring goal: no pose received from the bus yet.\n";
            return;
        }
        double inf = kInflateM;
        if (json_number(msg, "inflate", inf)) inf = std::clamp(inf, 0.0, 1.5);
        else inf = kInflateM;
        inflate_m = inf;

        const Pose cur = get_pose();
        route = std::move(wps);
        active_goal = route.back();
        is_teleop_active = false;
        std::cout << "[Planner] New " << (route.size() > 1 ? "route with " + std::to_string(route.size()) + " waypoints" : std::string("goal"))
                  << " -> (" << active_goal.x << ", " << active_goal.y << ") from (" << cur.x << ", " << cur.y
                  << "), keep-out " << std::fixed << std::setprecision(2) << inflate_m << " m\n";

        if (!plan_and_publish(cur, true, true)) {
            has_active_goal = false;
            clear_path();
            stop_motors(false);
            send_cmd(0.0, 0.0);
            return;
        }
        has_active_goal = true;
        last_replan = now;
    };

    auto handle_goal = [&](const std::string& msg, const std::string& action, Clock::time_point now) {
        if (action == "teleop" || action == "key") {
            is_teleop_active = true;
            has_active_goal = false;
            clear_path();
        } else if (action == "cancel") {
            std::cout << "[Bus] Goal cancelled.\n";
            has_active_goal = false;
            is_teleop_active = false;
            clear_path();
            stop_motors(false);
            send_cmd(0.0, 0.0);
        } else if (action == "reset_map") {
            world_map.reset();
            std::cout << "[Map] Cleared by request.\n";
        } else if (action == "go") {
            double gx, gy;
            if (!json_number(msg, "x", gx) || !json_number(msg, "y", gy)) {
                std::cerr << "[Planner] Ignoring 'go' without x and y.\n";
                return;
            }
            start_route(msg, {Pose{gx, gy, 0.0}}, now);
        } else if (action == "route") {
            std::vector<Pose> wps;
            if (!json_xy_list(msg, "route", wps)) {
                std::cerr << "[Planner] Ignoring 'route' with no usable waypoints.\n";
                return;
            }
            start_route(msg, std::move(wps), now);
        }
    };

    auto handle_lidar = [&](const std::string& msg, Clock::time_point now) {
        const size_t p = msg.find("\"points\"");
        if (p == std::string::npos) return;
        const size_t outer = msg.find('[', p);
        if (outer == std::string::npos) return;

        std::vector<Ray> hits;
        std::vector<char> seen(360, 0);
        size_t returns = 0, close_returns = 0;

        size_t cur = outer + 1;
        while (true) {
            const size_t nb = msg.find('[', cur);
            const size_t ne = msg.find(']', cur);
            if (ne == std::string::npos) break;
            if (nb == std::string::npos || ne < nb) break;   // hit the closing bracket of the whole list

            const char* s = msg.c_str() + nb + 1;
            char* e1 = nullptr;
            const double angle_deg = std::strtod(s, &e1);
            if (e1 != s) {
                while (*e1 == ' ' || *e1 == ',') ++e1;
                char* e2 = nullptr;
                const double dist_mm = std::strtod(e1, &e2);
                if (e2 != e1 && dist_mm > 0.0) {
                    const double dist_m = dist_mm / 1000.0;
                    ++returns;
                    if (dist_m < 0.3) ++close_returns;
                    const int bin = ((static_cast<int>(std::lround(angle_deg)) % 360) + 360) % 360;
                    seen[bin] = 1;   // a beam came back here, so it is not "open ground"
                    if (dist_m >= kLidarMinM && dist_m <= kLidarMaxM) {
                        hits.push_back({dist_m, -angle_deg * M_PI / 180.0});
                    }
                }
            }
            cur = ne + 1;
        }

        double ts;
        if (!json_number(msg, "ts", ts) || ts <= 0.0) ts = now_wall();

        PoseHistory::Sample sp{};
        double yaw_rate = 0.0;
        const bool have_scan_pose = history.lookup(ts, sp, yaw_rate);
        const Pose scan_pose = have_scan_pose ? Pose{sp.x, sp.y, sp.yaw} : get_pose();

        // Forward clearance + stop logic always runs, on the best pose available
        local_costmap.update(hits, scan_pose.x, scan_pose.y, scan_pose.yaw);
        forward_clearance = computeForwardClearance(local_costmap, scan_pose.yaw);
        last_scan_rx = now;
        have_scan = true;

        // The world map only takes scans it can place reliably
        const bool bad_scan = returns > 0 && (static_cast<double>(close_returns) / returns) > 0.6;
        if (!have_pose || !have_scan_pose || bad_scan) return;
        if (hits.empty() && !kInferFreeFromMissing) return;   // real hardware: an empty scan means a dead lidar, not open ground
        if (std::abs(yaw_rate) > kMapMaxYawRate) return;

        world_map.recenterIfNeeded(scan_pose.x, scan_pose.y);
        world_map.clearFootprint(scan_pose.x, scan_pose.y, kFootprintM);

        std::vector<Ray> free_rays;
        if (kInferFreeFromMissing) {
            for (int b = 0; b < 360; ++b) {
                bool any = false;
                for (int k = -2; k <= 2 && !any; ++k) any = seen[(b + k + 360) % 360] != 0;
                if (!any) free_rays.push_back({kFreeRayM, -b * M_PI / 180.0});
            }
        }
        // Heading: position is RTK, so only the yaw can be off. Line the scan up with what the map already holds
        // and put it in at the matched heading, so a drifting heading cannot smear a second copy of every wall.
        double map_yaw = wrap_angle(scan_pose.yaw + heading_bias);
        {
            double matched = 0.0, fit = 0.0;
            if (world_map.matchYaw(scan_pose.x, scan_pose.y, map_yaw, hits, matched, fit)) {
                const double total = wrap_angle(matched - scan_pose.yaw);
                if (std::abs(wrap_angle(matched - map_yaw)) > 0.15 * M_PI / 180.0) {
                    heading_bias = std::clamp(heading_bias + 0.5 * wrap_angle(total - heading_bias), -kMaxBiasRad, kMaxBiasRad);
                    map_yaw = wrap_angle(matched);
                }
                if (std::abs(heading_bias - logged_bias) > 0.5 * M_PI / 180.0) {
                    logged_bias = heading_bias;
                    std::cout << "[Map] heading correction " << std::fixed << std::setprecision(1) << heading_bias * 180.0 / M_PI
                              << " deg (scan fit " << std::setprecision(2) << fit << ")\n" << std::defaultfloat;
                }
            }
        }
        world_map.integrate(scan_pose.x, scan_pose.y, map_yaw, hits, free_rays);
    };

    std::cout << "\n[Main] Rover online and waiting for commands...\n\n";

    // ---------------------------------------------------------------- one 20 Hz iteration
    auto step = [&]() {
        const auto now = Clock::now();

        // 1. Incoming messages. Poses first so every scan in this batch can find its pose.
        struct Item { MsgKind kind; std::string action; std::string text; };
        std::vector<Item> items;
        for (auto& m : udp_bus.poll_messages()) {
            Item it;
            it.kind = classify(m, it.action);
            if (it.kind == MsgKind::Other) continue;
            it.text = std::move(m);
            items.push_back(std::move(it));
        }
        for (const auto& it : items) if (it.kind == MsgKind::Pose)  handle_pose(it.text, now);
        for (const auto& it : items) if (it.kind == MsgKind::Goal)  handle_goal(it.text, it.action, now);
        for (const auto& it : items) if (it.kind == MsgKind::Lidar) handle_lidar(it.text, now);

        // 2. Control
        const Pose cur = get_pose();
        const double pose_age = have_pose ? secs_between(now, last_pose_rx) : 1e9;

        if (is_teleop_active) {
            g_mode = "TELEOP";
        } else if (has_active_goal) {
            g_mode = "NAVIGATING";
            const bool pose_ok  = have_pose && pose_age <= kPoseTimeoutS;
            const bool lidar_ok = !kRequireFreshLidar || (have_scan && secs_between(now, last_scan_rx) <= kScanTimeoutS);

            if (!pose_ok) {
                if (due(warn_pose, now, 2.0)) std::cerr << "[Nav] No fresh pose from the bus (" << pose_age << " s). Holding.\n";
                {
                    std::lock_guard<std::mutex> lock(state.state_mutex);
                    state.velocity = {0.0, 0.0};
                    state.motor_command.linear = 0.0;
                    state.motor_command.angular = 0.0;
                    state.motor_command.enabled = true;
                }
                send_cmd(0.0, 0.0);
            } else {
                // Waypoints the rover has reached drop off the front of the route (the last one is the goal itself)
                while (route.size() > 1 && std::hypot(route.front().x - cur.x, route.front().y - cur.y) < kWaypointTolM) {
                    std::cout << "[Route] Waypoint reached (" << route.front().x << ", " << route.front().y << "), "
                              << route.size() - 1 << " to go\n";
                    route.erase(route.begin());
                }

                if (secs_between(now, last_replan) >= kReplanPeriodS) {
                    last_replan = now;
                    if (!plan_and_publish(cur, false, false) && due(warn_plan, now, 3.0)) {
                        std::cerr << "[Planner] Replan failed; keeping the current path.\n";
                    }
                }

                const auto result = follower.update(cur.x, cur.y, cur.yaw);
                Velocity cmd = result.first;
                const bool done = result.second;

                if (done) {
                    std::cout << "[Planner] Destination reached. Holding position.\n";
                    has_active_goal = false;
                    is_teleop_active = false;
                    clear_path();
                    stop_motors(false);
                    send_cmd(0.0, 0.0);
                    zero_repeats = 2;
                } else {
                    double scale = 1.0;
                    if (!lidar_ok) {
                        scale = 0.0;
                        if (due(warn_lidar, now, 2.0)) std::cerr << "[Nav] No fresh lidar scan. Holding.\n";
                    } else if (forward_clearance < 0.35) {
                        scale = 0.0;
                    } else if (forward_clearance < 1.0) {
                        scale = (forward_clearance - 0.35) / 0.65;
                    }
                    if (cmd.linear > 0.0) cmd.linear *= scale;

                    // Corners: same arc, driven slower. Fast turns smear the map (pose error grows with turn rate).
                    const double aw = std::abs(cmd.angular);
                    if (aw > kTurnSlowW) {
                        const double ts = std::max(kTurnSlowMin, 1.0 - (1.0 - kTurnSlowMin) * (aw - kTurnSlowW) / (1.0 - kTurnSlowW));
                        cmd.linear *= ts;
                        cmd.angular *= ts;
                    }

                    {
                        std::lock_guard<std::mutex> lock(state.state_mutex);
                        state.velocity = cmd;
                        state.motor_command.linear = cmd.linear;
                        state.motor_command.angular = cmd.angular;
                        state.motor_command.enabled = true;
                    }
                    send_cmd(cmd.linear, cmd.angular);
                }
            }
        } else {
            g_mode = "IDLE";
            stop_motors(false);
            if (zero_repeats > 0) {
                send_cmd(0.0, 0.0);
                --zero_repeats;
            }
        }

        // 3. Telemetry once a second
        if (secs_between(now, last_log) >= 1.0) {
            last_log = now;
            Velocity v;
            {
                std::lock_guard<std::mutex> lock(state.state_mutex);
                v = state.velocity;
            }
            std::cout << "[NAV] Pose: (" << std::fixed << std::setprecision(2) << cur.x << ", " << cur.y << ") m"
                      << " | Yaw: " << cur.yaw << " rad"
                      << " | Mode: " << g_mode.load()
                      << " | v: " << v.linear << " w: " << v.angular
                      << " | front: " << forward_clearance << " m"
                      << " | route: " << route.size()
                      << " | pose age: " << std::min(pose_age, 99.0) << " s"
                      << " | walls: " << world_map.occupiedCount() << "\n";
        }
    };

    // ---------------------------------------------------------------- main loop
    // The try/catch is INSIDE the loop: one bad iteration is reported and skipped, it does not end the program.
    const auto period = std::chrono::milliseconds(50);
    auto next_tick = Clock::now();
    int consecutive_errors = 0;

    while (g_running) {
        next_tick += period;
        try {
            step();
            consecutive_errors = 0;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in main loop: " << e.what() << "\n";
            if (++consecutive_errors >= 50) { std::cerr << "[ERROR] Too many consecutive errors, shutting down.\n"; break; }
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in main loop.\n";
            if (++consecutive_errors >= 50) { std::cerr << "[ERROR] Too many consecutive errors, shutting down.\n"; break; }
        }
        std::this_thread::sleep_until(next_tick);
        if (Clock::now() > next_tick + std::chrono::milliseconds(500)) next_tick = Clock::now();   // fell behind: do not burst
    }

    std::cout << "[Main] Shutting down.\n";
    stop_motors(false);
    send_cmd(0.0, 0.0);
    motor_driver.stop();
    debug_server.stop();
    return 0;
}
