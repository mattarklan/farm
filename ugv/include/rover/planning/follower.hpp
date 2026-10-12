#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>

namespace rover::planning {

class PurePursuit {
public:
    PurePursuit(double max_speed, double max_turn, double lookahead = 0.6, double goal_tol = 0.25)
        : max_speed_(max_speed), max_turn_(max_turn), lookahead_(lookahead), goal_tol_(goal_tol), seg_(0) {}

    void set_path(const std::vector<::Pose>& path) {
        path_ = path;
        seg_ = 0;
    }

    // Returns std::pair<::Velocity, bool done>
    std::pair<::Velocity, bool> update(double x, double y, double yaw) {
        ::Velocity cmd{0.0, 0.0};
        if (path_.empty()) {
            return {cmd, true};
        }

        const auto& goal = path_.back();
        double dist = std::hypot(goal.x - x, goal.y - y);
        if (dist < goal_tol_) {
            return {cmd, true};
        }

        auto [tx, ty] = get_target(x, y);
        double alpha = wrap(std::atan2(ty - y, tx - x) - yaw);
        
        // Scale angular velocity sharply to swing around toward the target
        double w = std::clamp(2.5 * alpha, -max_turn_, max_turn_);

        // Bulb/Arc turn behavior: Allow a gentle forward creep (0.05) even when facing away,
        // forcing a smooth arc turn instead of getting stuck or stalling on goals behind it.
        double heading_factor = std::cos(alpha);
        double v = max_speed_ * std::max(0.05, heading_factor * heading_factor);

        v = std::min(v, 0.15 + 1.0 * dist); // ease off near goal
        v *= (1.0 - 0.4 * std::abs(w) / max_turn_); // slow down in tight turns

        cmd.linear = v;
        cmd.angular = w;
        return {cmd, false};
    }

private:
    double wrap(double a) const {
        return std::fmod(a + M_PI, 2.0 * M_PI) - M_PI;
    }

    std::pair<double, double> get_target(double x, double y) {
        if (path_.size() < 2) {
            return {path_.back().x, path_.back().y};
        }

        double best_d = 1e9;
        size_t best_i = seg_;
        double best_t = 0.0;

        for (size_t i = seg_; i < path_.size() - 1; ++i) {
            double ax = path_[i].x, ay = path_[i].y;
            double bx = path_[i+1].x, by = path_[i+1].y;
            double dx = bx - ax, dy = by - ay;
            double L2 = dx * dx + dy * dy;
            double t = (L2 == 0.0) ? 0.0 : std::clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0);
            double d = std::hypot(x - (ax + t * dx), y - (ay + t * dy));

            if (d < best_d - 1e-9) {
                best_d = d;
                best_i = i;
                best_t = t;
            } else if (i > best_i + 1 && d > best_d + 2 * lookahead_) {
                break;
            }
        }

        seg_ = best_i;
        double ax = path_[seg_].x, ay = path_[seg_].y;
        double bx = path_[seg_+1].x, by = path_[seg_+1].y;
        double px = ax + best_t * (bx - ax);
        double py = ay + best_t * (by - ay);

        double remaining = lookahead_;
        while (true) {
            double seg_len = std::hypot(bx - px, by - py);
            if (seg_len >= remaining) {
                double f = (seg_len > 0.0) ? (remaining / seg_len) : 0.0;
                return {px + f * (bx - px), py + f * (by - py)};
            }
            remaining -= seg_len;
            seg_++;
            if (seg_ >= path_.size() - 1) {
                return {path_.back().x, path_.back().y};
            }
            px = path_[seg_].x;
            py = path_[seg_].y;
            bx = path_[seg_+1].x;
            by = path_[seg_+1].y;
        }
    }

    double max_speed_;
    double max_turn_;
    double lookahead_;
    double goal_tol_;
    std::vector<::Pose> path_;
    size_t seg_;
};

} // namespace rover::planning