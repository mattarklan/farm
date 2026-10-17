#include "rover/planning/theta_star.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <queue>
#include <vector>

namespace rover::planning {

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

bool ThetaStarPlanner::find_nearest_free(int& x, int& y, const std::vector<uint8_t>& grid, int max_radius) const {
    if (grid[y * width_ + x] == 0) return true;

    for (int r = 1; r <= max_radius; ++r) {
        double best_d2 = 1e18;
        int bx = -1, by = -1;
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
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
        std::cerr << "[Theta* ERROR] Start or Goal is outside the map!\n";
        return path;
    }

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

    // The open list stores a SNAPSHOT of f = g + h taken when the node is pushed.
    // The old queue compared nodes by their live g_cost, which Theta* changes while the node is
    // still inside the heap. That silently breaks the heap ordering and produces odd, longer paths.
    struct Entry {
        double f;
        GridNode* node;
    };
    struct EntryCmp {
        bool operator()(const Entry& a, const Entry& b) const { return a.f > b.f; }   // min-heap on f
    };
    std::priority_queue<Entry, std::vector<Entry>, EntryCmp> open_set;

    GridNode* start_node = get_node(start_gx, start_gy);
    start_node->g_cost = 0.0;
    open_set.push({start_node->g_cost + start_node->h_cost, start_node});

    static const int neighbors[8][2] = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    GridNode* goal_found = nullptr;
    int expansions = 0;
    // Enough to search most of the 600x600 map. A big detour around a real wall needs far more
    // than the old 40,000, which is why open-looking routes sometimes came back as "no path".
    const int max_expansions = 250000;

    while (!open_set.empty()) {
        if (++expansions > max_expansions) {
            std::cerr << "[Theta* ERROR] Path search aborted: exceeded max expansions (unreachable goal?).\n";
            return path;
        }

        const Entry top = open_set.top();
        open_set.pop();
        GridNode* current = top.node;

        const int current_key = current->y * width_ + current->x;
        if (closed[current_key]) continue;
        closed[current_key] = 1;

        if (current->x == goal_gx && current->y == goal_gy) {
            goal_found = current;
            break;
        }

        for (const auto& dir : neighbors) {
            int nx = current->x + dir[0];
            int ny = current->y + dir[1];

            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

            int n_key = ny * width_ + nx;
            if (closed[n_key]) continue;
            if (grid[n_key] != 0) continue;

            if (dir[0] != 0 && dir[1] != 0) {
                if (grid[current->y * width_ + nx] != 0 || grid[ny * width_ + current->x] != 0) continue;
            }

            GridNode* neighbor = get_node(nx, ny);

            if (current->parent && line_of_sight(current->parent, neighbor, grid)) {
                double new_g = current->parent->g_cost + EuclideanDistance(current->parent, neighbor);
                if (new_g < neighbor->g_cost) {
                    neighbor->g_cost = new_g;
                    neighbor->parent = current->parent;
                    open_set.push({new_g + neighbor->h_cost, neighbor});
                }
            } else {
                double new_g = current->g_cost + EuclideanDistance(current, neighbor);
                if (new_g < neighbor->g_cost) {
                    neighbor->g_cost = new_g;
                    neighbor->parent = current;
                    open_set.push({new_g + neighbor->h_cost, neighbor});
                }
            }
        }
    }

    if (!goal_found) {
        std::cerr << "[Theta*] No path found to goal.\n";
        return path;
    }

    for (GridNode* curr = goal_found; curr; curr = curr->parent) {
        path.push_back(grid_to_world(curr->x, curr->y));
    }
    std::reverse(path.begin(), path.end());

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
