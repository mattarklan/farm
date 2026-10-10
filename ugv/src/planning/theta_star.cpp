#include "rover/planning/theta_star.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

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

// Map world ENU coordinates to cell indices relative to grid origin (0,0 mapped to map center)
bool ThetaStarPlanner::world_to_grid(double wx, double wy, const Pose& start_pose, int& gx, int& gy) const {
    // Offset relative to starting pose, placing start_pose near the center of the grid (width/2, height/2)
    double rel_x = wx - start_pose.x + (width_ * resolution_ / 2.0);
    double rel_y = wy - start_pose.y + (height_ * resolution_ / 2.0);

    gx = static_cast<int>(std::floor(rel_x / resolution_));
    gy = static_cast<int>(std::floor(rel_y / resolution_));

    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

Pose ThetaStarPlanner::grid_to_world(int gx, int gy, const Pose& start_pose) const {
    Pose p;
    p.x = (static_cast<double>(gx) + 0.5) * resolution_ - (width_ * resolution_ / 2.0) + start_pose.x;
    p.y = (static_cast<double>(gy) + 0.5) * resolution_ - (height_ * resolution_ / 2.0) + start_pose.y;
    p.yaw = 0.0;
    return p;
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

        int idx = y0 * width_ + x0;
        if (idx >= 0 && idx < static_cast<int>(grid.size())) {
            if (grid[idx] != 0) return false;
        }

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            x0 += sy;
        }
    }
    return true;
}

std::vector<Pose> ThetaStarPlanner::plan_path(const Pose& start_pose, 
                                            const Pose& goal_pose, 
                                            const std::vector<uint8_t>& grid) 
{
    std::vector<Pose> path;

    int start_gx, start_gy, goal_gx, goal_gy;
    if (!world_to_grid(start_pose.x, start_pose.y, start_pose, start_gx, start_gy) ||
        !world_to_grid(goal_pose.x, goal_pose.y, start_pose, goal_gx, goal_gy)) {
        std::cerr << "[Theta* ERROR] Start or Goal coordinates mapped out of bounds!\n"
                  << "  Start Pose: (" << start_pose.x << ", " << start_pose.y << ") -> Grid: (" << start_gx << ", " << start_gy << ")\n"
                  << "  Goal Pose:  (" << goal_pose.x << ", " << goal_pose.y << ") -> Grid: (" << goal_gx << ", " << goal_gy << ")\n";
        return path;
    }

    // Verify start or goal are not inside an obstacle
    int start_idx = start_gy * width_ + start_gx;
    int goal_idx  = goal_gy * width_ + goal_gx;
    if (grid[start_idx] != 0 || grid[goal_idx] != 0) {
        std::cerr << "[Theta* ERROR] Start or Goal node coincides with an occupied obstacle cell!\n";
        return path;
    }

    std::priority_queue<GridNode*, std::vector<GridNode*>, CompareNode> open_set;
    std::unordered_map<int, GridNode*> all_nodes;
    std::unordered_map<int, bool> closed_set;

    auto get_node_key = [this](int x, int y) { return y * width_ + x; };

    GridNode* start_node = new GridNode{start_gx, start_gy, 0.0, 0.0, nullptr};
    GridNode* goal_node  = new GridNode{goal_gx, goal_gy, 1e9, 0.0, nullptr};

    start_node->h_cost = Heuristic(start_node, goal_node);
    open_set.push(start_node);
    all_nodes[get_node_key(start_gx, start_gy)] = start_node;

    static const int neighbors[8][2] = {
        {1,0}, {-1,0}, {0,1}, {0,-1},
        {1,1}, {1,-1}, {-1,1}, {-1,-1}
    };

    GridNode* current = nullptr;
    while (!open_set.empty()) {
        current = open_set.top();
        open_set.pop();

        int current_key = get_node_key(current->x, current->y);
        if (closed_set[current_key]) continue;
        closed_set[current_key] = true;

        if (current->x == goal_gx && current->y == goal_gy) {
            break;
        }

        for (const auto& dir : neighbors) {
            int nx = current->x + dir[0];
            int ny = current->y + dir[1];

            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

            int n_key = get_node_key(nx, ny);
            if (closed_set[n_key]) continue;

            if (n_key >= 0 && n_key < static_cast<int>(grid.size()) && grid[n_key] != 0) {
                continue;
            }

            GridNode* neighbor = nullptr;
            if (all_nodes.find(n_key) == all_nodes.end()) {
                neighbor = new GridNode{nx, ny, 1e9, 0.0, nullptr};
                neighbor->h_cost = Heuristic(neighbor, goal_node);
                all_nodes[n_key] = neighbor;
            } else {
                neighbor = all_nodes[n_key];
            }

            if (current->parent && line_of_sight(current->parent, neighbor, grid)) {
                double new_g = current->parent->g_cost + EuclideanDistance(current->parent, neighbor);
                if (new_g < neighbor->g_cost) {
                    neighbor->g_cost = new_g;
                    neighbor->parent = current->parent;
                    open_set.push(neighbor);
                }
            } else {
                double new_g = current->g_cost + EuclideanDistance(current, neighbor);
                if (new_g < neighbor->g_cost) {
                    neighbor->g_cost = new_g;
                    neighbor->parent = current;
                    open_set.push(neighbor);
                }
            }
        }
    }

    if (current && current->x == goal_gx && current->y == goal_gy) {
        GridNode* curr = current;
        while (curr) {
            path.push_back(grid_to_world(curr->x, curr->y, start_pose));
            curr = curr->parent;
        }
        std::reverse(path.begin(), path.end());

        for (size_t i = 0; i < path.size(); ++i) {
            if (i + 1 < path.size()) {
                path[i].yaw = std::atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x);
            } else if (i > 0) {
                path[i].yaw = path[i-1].yaw;
            }
        }
    }

    for (auto& pair : all_nodes) {
        delete pair.second;
    }
    delete goal_node;

    return path;
}

} // namespace rover::planning