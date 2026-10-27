#include "rover/planning/theta_star.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>

namespace rover::planning {

namespace {
// Priority-queue entry with the f-cost frozen at push time. (Comparing live node costs inside the heap,
// as the old code did, breaks the heap ordering whenever a queued node's cost is later lowered.)
struct QEntry {
    double f;
    GridNode* n;
};
struct QCmp {
    bool operator()(const QEntry& a, const QEntry& b) const { return a.f > b.f; }
};
} // namespace

ThetaStarPlanner::ThetaStarPlanner(int map_width_cells, int map_height_cells, double resolution_m)
    : width_(map_width_cells), height_(map_height_cells), resolution_(resolution_m) {}

double ThetaStarPlanner::EuclideanDistance(const GridNode* a, const GridNode* b) const {
    double dx = static_cast<double>(a->x - b->x) * resolution_;
    double dy = static_cast<double>(a->y - b->y) * resolution_;
    return std::hypot(dx, dy);
}

double ThetaStarPlanner::Heuristic(const GridNode* a, const GridNode* b) const {
    return EuclideanDistance(a, b);
}

// World ENU -> cell index. The grid is anchored at a FIXED world origin (set_origin), so cells
// keep their world position as the rover drives around.
bool ThetaStarPlanner::world_to_grid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

Pose ThetaStarPlanner::grid_to_world(int gx, int gy) const {
    Pose p;
    p.x = origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_;
    p.y = origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_;
    p.yaw = 0.0;
    return p;
}

// If (x,y) is blocked, move it to the closest free cell within max_radius cells.
bool ThetaStarPlanner::find_nearest_free(int& x, int& y, const std::vector<uint8_t>& grid, int max_radius) const {
    if (grid[y * width_ + x] == 0) return true;

    for (int r = 1; r <= max_radius; ++r) {
        double best_d2 = 1e18;
        int bx = -1, by = -1;
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue; // ring only
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
                if (grid[ny * width_ + nx] != 0) continue;
                double d2 = static_cast<double>(dx * dx + dy * dy);
                if (d2 < best_d2) { best_d2 = d2; bx = nx; by = ny; }
            }
        }
        if (bx >= 0) { x = bx; y = by; return true; }
    }
    return false;
}

bool ThetaStarPlanner::line_of_sight(const GridNode* s, const GridNode* s_prime, const std::vector<uint8_t>& grid) const {
    int x0 = s->x, y0 = s->y;
    int x1 = s_prime->x, y1 = s_prime->y;
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 < 0 || x0 >= width_ || y0 < 0 || y0 >= height_) return false;
        if (grid[y0 * width_ + x0] != 0) return false;
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        bool step_x = e2 > -dy;
        bool step_y = e2 < dx;

        // Diagonal step: don't squeeze between two touching obstacle corners
        if (step_x && step_y) {
            int ax = x0 + sx, ay = y0 + sy;
            if (ax < 0 || ax >= width_ || ay < 0 || ay >= height_) return false;
            if (grid[y0 * width_ + ax] != 0 || grid[ay * width_ + x0] != 0) return false;
        }

        if (step_x) { err -= dy; x0 += sx; }
        if (step_y) { err += dx; y0 += sy; }
    }
    return true;
}

// Lazy Theta*.
//
// Plain Theta* runs a line-of-sight check from every expanded node's PARENT to EVERY neighbour it touches
// (up to 8 per expansion). Lazy Theta* optimistically ASSUMES that line of sight exists when a neighbour is
// first reached, and only checks it when the node is actually popped for expansion (SetVertex). Most nodes
// that get reached are never popped, so most of those checks never happen. If the check fails when the node
// is popped, its parent is repaired from an already-closed grid neighbour. Paths are still any-angle, and
// every edge in the returned path has been verified against the grid.
std::vector<Pose> ThetaStarPlanner::plan_path(const Pose& start_pose,
                                            const Pose& goal_pose,
                                            const std::vector<uint8_t>& grid)
{
    std::vector<Pose> path;

    if (static_cast<int>(grid.size()) != width_ * height_) {
        std::cerr << "[Theta* ERROR] Grid size does not match planner dimensions!\n";
        return path;
    }

    int start_gx, start_gy, goal_gx, goal_gy;
    if (!world_to_grid(start_pose.x, start_pose.y, start_gx, start_gy) ||
        !world_to_grid(goal_pose.x, goal_pose.y, goal_gx, goal_gy)) {
        std::cerr << "[Theta* ERROR] Start or Goal is outside the map!\n"
                  << "  Start: (" << start_pose.x << ", " << start_pose.y << ") -> cell (" << start_gx << ", " << start_gy << ")\n"
                  << "  Goal:  (" << goal_pose.x << ", " << goal_pose.y << ") -> cell (" << goal_gx << ", " << goal_gy << ")\n";
        return path;
    }

    // Start/goal inside an obstacle or its inflation: snap to the nearest free cell rather than failing.
    if (!find_nearest_free(start_gx, start_gy, grid, 8)) {
        std::cerr << "[Theta* ERROR] No free cell near the start position!\n";
        return path;
    }
    const int orig_goal_gx = goal_gx, orig_goal_gy = goal_gy;
    if (!find_nearest_free(goal_gx, goal_gy, grid, 15)) {
        std::cerr << "[Theta* ERROR] Goal is inside an obstacle and no free cell is nearby!\n";
        return path;
    }
    const bool goal_moved = (goal_gx != orig_goal_gx || goal_gy != orig_goal_gy);
    if (goal_moved) {
        std::cerr << "[Theta*] Goal was blocked; moved to nearest free cell.\n";
    }

    const int N = width_ * height_;
    std::vector<GridNode*> nodes(N, nullptr);
    std::vector<uint8_t> closed(N, 0);
    std::vector<std::unique_ptr<GridNode>> pool;

    auto get_node = [&](int x, int y) -> GridNode* {
        int k = y * width_ + x;
        if (!nodes[k]) {
            pool.emplace_back(new GridNode{x, y, 1e9, 0.0, nullptr});
            GridNode* n = pool.back().get();
            n->h_cost = std::hypot(static_cast<double>(x - goal_gx), static_cast<double>(y - goal_gy)) * resolution_;
            nodes[k] = n;
        }
        return nodes[k];
    };

    static const int neighbors[8][2] = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    // A grid step from (x,y) by dir is allowed if it stays on the map, lands on a free cell and (for
    // diagonals) does not cut between two touching obstacle corners.
    auto step_ok = [&](int x, int y, const int* dir) {
        int nx = x + dir[0], ny = y + dir[1];
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) return false;
        if (grid[ny * width_ + nx] != 0) return false;
        if (dir[0] != 0 && dir[1] != 0) {
            if (grid[y * width_ + nx] != 0 || grid[ny * width_ + x] != 0) return false;
        }
        return true;
    };

    // SetVertex: the optimistic parent of s was never checked. Check it now; if it has no line of sight,
    // fall back to the best already-closed grid neighbour (whose own parent chain has been verified).
    auto set_vertex = [&](GridNode* s) {
        if (s->parent == s) return; // start node
        if (line_of_sight(s->parent, s, grid)) return;

        double best = 1e18;
        GridNode* best_parent = nullptr;
        for (const auto& dir : neighbors) {
            if (!step_ok(s->x, s->y, dir)) continue;
            int k = (s->y + dir[1]) * width_ + (s->x + dir[0]);
            if (!closed[k]) continue;
            GridNode* c = nodes[k];
            double g = c->g_cost + EuclideanDistance(c, s);
            if (g < best) { best = g; best_parent = c; }
        }
        if (best_parent) {
            s->parent = best_parent;
            s->g_cost = best;
        }
    };

    std::priority_queue<QEntry, std::vector<QEntry>, QCmp> open_set;

    GridNode* start_node = get_node(start_gx, start_gy);
    start_node->g_cost = 0.0;
    start_node->parent = start_node; // marks the root of the parent chain
    open_set.push({start_node->h_cost, start_node});

    GridNode* goal_found = nullptr;
    while (!open_set.empty()) {
        GridNode* current = open_set.top().n;
        open_set.pop();

        const int current_key = current->y * width_ + current->x;
        if (closed[current_key]) continue; // stale duplicate entry

        set_vertex(current);

        if (current->x == goal_gx && current->y == goal_gy) {
            goal_found = current;
            break;
        }
        closed[current_key] = 1;

        // ComputeCost: a neighbour optimistically inherits current's (now verified) parent
        GridNode* p = current->parent;
        for (const auto& dir : neighbors) {
            if (!step_ok(current->x, current->y, dir)) continue;
            int nx = current->x + dir[0];
            int ny = current->y + dir[1];
            if (closed[ny * width_ + nx]) continue;

            GridNode* neighbor = get_node(nx, ny);
            double new_g = p->g_cost + EuclideanDistance(p, neighbor);
            if (new_g < neighbor->g_cost) {
                neighbor->g_cost = new_g;
                neighbor->parent = p;
                open_set.push({new_g + neighbor->h_cost, neighbor});
            }
        }
    }

    if (!goal_found) {
        std::cerr << "[Theta*] No path found to goal.\n";
        return path;
    }

    for (GridNode* curr = goal_found;; curr = curr->parent) {
        path.push_back(grid_to_world(curr->x, curr->y));
        if (curr->parent == curr) break;
    }
    std::reverse(path.begin(), path.end());

    // Use the exact start pose / goal point instead of cell centers
    path.front().x = start_pose.x;
    path.front().y = start_pose.y;
    if (!goal_moved) {
        if (path.size() == 1) {
            path.push_back(path.front());
        }
        path.back().x = goal_pose.x;
        path.back().y = goal_pose.y;
    } else if (path.size() == 1) {
        path.push_back(grid_to_world(goal_gx, goal_gy));
    }

    for (size_t i = 0; i < path.size(); ++i) {
        if (i + 1 < path.size()) {
            path[i].yaw = std::atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x);
        } else if (i > 0) {
            path[i].yaw = path[i-1].yaw;
        }
    }

    return path;
}

} // namespace rover::planning
