#pragma once

#include "rover/common/types.hpp"
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

    std::vector<Pose> plan_path(const Pose& start_pose, 
                                const Pose& goal_pose, 
                                const std::vector<uint8_t>& occupancy_grid);

private:
    bool line_of_sight(const GridNode* s, const GridNode* s_prime, const std::vector<uint8_t>& grid) const;
    double EuclideanDistance(const GridNode* a, const GridNode* b) const;
    double Heuristic(const GridNode* a, const GridNode* b) const;

    bool world_to_grid(double wx, double wy, const Pose& start_pose, int& gx, int& gy) const;
    Pose grid_to_world(int gx, int gy, const Pose& start_pose) const;

    int width_;
    int height_;
    double resolution_;
};

} // namespace rover::planning