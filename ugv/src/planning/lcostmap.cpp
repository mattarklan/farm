#include "rover/planning/lcostmap.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

LocalCostmap::LocalCostmap(int width_cells, int height_cells, double resolution)
    : width_(width_cells), height_(height_cells), resolution_(resolution) {
    grid_.resize(width_ * height_, {0});
}

bool LocalCostmap::worldToGrid(double wx, double wy, double rx, double ry, int& gx_out, int& gy_out) const {
    double dx = wx - rx;
    double dy = wy - ry;

    int gx = static_cast<int>(std::floor(dx / resolution_)) + width_ / 2;
    int gy = static_cast<int>(std::floor(dy / resolution_)) + height_ / 2;

    gx_out = gx;
    gy_out = gy;

    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

void LocalCostmap::rayTrace(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
        if (x == x1 && y == y1) break;

        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            int idx = y * width_ + x;
            if (grid_[idx].cost < 100) {
                grid_[idx].cost = 0;
            }
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

void LocalCostmap::update(const std::vector<std::pair<double, double>>& lidar_scans, double rx, double ry, double ryaw) {
    std::fill(grid_.begin(), grid_.end(), Cell{0});

    int rx_idx = width_ / 2;
    int ry_idx = height_ / 2;

    for (const auto& scan : lidar_scans) {
        double range = scan.first;
        double angle = scan.second;

        if (range <= 0.0) continue;

        double wx = rx + range * std::cos(angle + ryaw);
        double wy = ry + range * std::sin(angle + ryaw);

        int gx, gy;
        if (worldToGrid(wx, wy, rx, ry, gx, gy)) {
            int idx = gy * width_ + gx;
            grid_[idx].cost = 100;

            rayTrace(rx_idx, ry_idx, gx, gy);
        }
    }

    inflateObstacles(3);
}

void LocalCostmap::inflateObstacles(int inflation_radius_cells) {
    std::vector<Cell> temp_grid = grid_;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            if (temp_grid[idx].cost == 100) {
                for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                    for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            int n_idx = ny * width_ + nx;
                            if (grid_[n_idx].cost < 100) {
                                grid_[n_idx].cost = 50;
                            }
                        }
                    }
                }
            }
        }
    }
}