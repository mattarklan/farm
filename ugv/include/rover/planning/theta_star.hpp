#pragma once

#include "rover/common/types.hpp"
#include <cstdint>
#include <vector>
#include <queue>
#include <unordered_map>

namespace rover::planning {

struct GridNode {
    int x;
    int y;
    double g_cost{1e9};
    double h_cost{0.0};
    GridNode* parent{nullptr};

    double f_cost() const { return g_cost + h_cost; }

    bool operator==(const GridNode& other) const {
        return x == other.x && y == other.y;
    }
};

struct CompareNode {
    bool operator()(const GridNode* a, const GridNode* b) const {
        return a->f_cost() > b->f_cost();
    }
};

class ThetaStarPlanner {
public:
    ThetaStarPlanner(int map_width_cells, int map_height_cells, double resolution_m);

    // World coordinates (m) of the corner of cell (0,0). The grid passed to plan_path()
    // MUST be anchored here and stay fixed in the world (not follow the rover).
    void set_origin(double origin_x, double origin_y) { origin_x_ = origin_x; origin_y_ = origin_y; }

    // grid: 0 = free, anything else = blocked (already inflated by the caller).
    std::vector<Pose> plan_path(const Pose& start_pose, 
                                const Pose& goal_pose, 
                                const std::vector<uint8_t>& occupancy_grid);

private:
    bool line_of_sight(const GridNode* s, const GridNode* s_prime, const std::vector<uint8_t>& grid) const;
    double EuclideanDistance(const GridNode* a, const GridNode* b) const;
    double Heuristic(const GridNode* a, const GridNode* b) const;

    bool world_to_grid(double wx, double wy, int& gx, int& gy) const;
    Pose grid_to_world(int gx, int gy) const;
    bool find_nearest_free(int& x, int& y, const std::vector<uint8_t>& grid, int max_radius) const;

    int width_;
    int height_;
    double resolution_;
    double origin_x_{0.0};
    double origin_y_{0.0};
};

} // namespace rover::planning