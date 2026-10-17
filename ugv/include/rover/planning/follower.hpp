#pragma once

#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>

namespace rover::planning {

class PurePursuit {
public:
    // min_turn_radius : tightest arc (m) used when the target is off to the side / behind
    // arc_speed       : minimum forward speed (m/s) kept while arcing, so the rover never spins on the spot
    PurePursuit(double max_speed, double max_turn, double lookahead = 0.6, double goal_tol = 0.25,
                double min_turn_radius = 0.5, double arc_speed = 0.3)
        : max_speed_(max_speed), max_turn_(max_turn), lookahead_(lookahead), goal_tol_(goal_tol),
          min_radius_(min_turn_radius), arc_speed_(arc_speed), seg_(0) {}

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

        const double abs_alpha = std::abs(alpha);

        // Forward speed: full speed when pointing at the target, falling off as the target
        // swings to the side. Targets at/behind 90 deg get 0 here, but we never drop below
        // arc_speed_, so the rover always drives forward along an arc (bulb turn) instead of
        // stalling or pivoting on the spot.
        double heading_factor = (abs_alpha < M_PI / 2.0) ? std::cos(alpha) : 0.0;
        double v = std::max(max_speed_ * heading_factor * heading_factor, arc_speed_);
        v = std::min(v, 0.15 + 1.0 * dist); // ease off near goal

        // Turning radius we are allowed to use. If the goal is behind us AND closer than the
        // turning circle, a normal arc would orbit the goal forever, so tighten the arc and slow down.
        double radius = min_radius_;
        if (abs_alpha > M_PI / 2.0 && dist < 2.0 * min_radius_) {
            radius = 0.15;
            v = std::min(v, 0.15);
        }

        // Angular rate is limited by v / radius, which is what makes the path a real arc.
        double w_limit = std::min(max_turn_, v / radius);
        double w = std::clamp(2.5 * alpha, -w_limit, w_limit);

        cmd.linear = v;
        cmd.angular = w;
        return {cmd, false};
    }

private:
    // Wrap to [-pi, pi).
    double wrap(double a) const {
        return a - 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI));
    }

    std::pair<double, double> get_target(double x, double y) {
        if (path_.size() < 2) {
            return {path_.back().x, path_.back().y};
        }

        const size_t last_seg = path_.size() - 2;      // largest valid segment index
        if (seg_ > last_seg) seg_ = last_seg;          // never let seg_ point past the last segment

        double best_d = 1e9;
        size_t best_i = seg_;
        double best_t = 0.0;

        for (size_t i = seg_; i <= last_seg; ++i) {
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

        // Walk the lookahead distance along the path from the closest point, using a LOCAL index so
        // seg_ (the persistent "where am I on the path" marker) always stays on a real segment.
        size_t k = seg_;
        double ax = path_[k].x, ay = path_[k].y;
        double bx = path_[k+1].x, by = path_[k+1].y;
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
            ++k;
            if (k > last_seg) {
                return {path_.back().x, path_.back().y};
            }
            px = path_[k].x;
            py = path_[k].y;
            bx = path_[k+1].x;
            by = path_[k+1].y;
        }
    }

    double max_speed_;
    double max_turn_;
    double lookahead_;
    double goal_tol_;
    double min_radius_;
    double arc_speed_;
    std::vector<::Pose> path_;
    size_t seg_;
};

} // namespace rover::planning
