#ifndef LCOSTMAP_HPP
#define LCOSTMAP_HPP

#include <vector>
#include <utility>

struct Cell {
    unsigned char cost{0}; // 0 = free space, 50 = inflated/danger, 100 = lethal obstacle
};

class LocalCostmap {
public:
    LocalCostmap(int width_cells, int height_cells, double resolution);

    bool worldToGrid(double wx, double wy, double rx, double ry, int& gx_out, int& gy_out) const;
    void rayTrace(int x0, int y0, int x1, int y1);
    void update(const std::vector<std::pair<double, double>>& lidar_scans, double rx, double ry, double ryaw);
    void inflateObstacles(int inflation_radius_cells);

    const std::vector<Cell>& getGrid() const { return grid_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<Cell> grid_;
};

#endif // LCOSTMAP_HPP