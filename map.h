#pragma once

#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include "utils.h"
namespace MapItemSpace
{
    enum class MapItem
    {
        SPACE = 0,
        SEA,
        OBSTACLE,
        ROBOT,
        BERTH,
        ERROR = -1,
    };
}

class Map
{
    // 地图坐标系原点在左上角，往下为 X 轴正方向，往右为 Y 轴正方向
public:
    int rows, cols;
    std::vector<std::vector<MapItemSpace::MapItem>> grid;

    static std::array<Point2d, 4> DIRS;

public:
    Map(int rows, int cols)
        : rows(rows),
          cols(cols),
          grid(std::vector(rows, std::vector<MapItemSpace::MapItem>(cols, MapItemSpace::MapItem::ERROR)))
    {
    }

    // 设置地图上某个位置的值
    void setCell(int x, int y, MapItemSpace::MapItem value)
    {
        if (x >= 0 && x < rows && y >= 0 && y < cols)
        {
            grid[x][y] = value;
        }
    }

    inline bool inBounds(int x, int y) const
    {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }
    inline bool inBounds(Point2d pos) const
    {
        return inBounds(pos.x, pos.y);
    }

    // 获取地图上某个位置的值
    inline MapItemSpace::MapItem getCell(int x, int y) const
    {
        return grid[x][y];
    }

    inline MapItemSpace::MapItem getCell(Point2d pos) const
    {
        return getCell(pos.x, pos.y);
    }

    inline int cost(Point2d pos1, Point2d pos2) const
    {
        return Point2d::calculateManhattanDistance(pos1, pos2);
    }
    
public:
    std::vector<Point2d> neighbors(Point2d id) const; // 返回当前节点上下左右的四个邻居
    bool passable(Point2d pos) const;
    // This outputs a grid. Pass in a distances map if you want to print
    // the distances, or pass in a point_to map if you want to print
    // arrows that point to the parent location, or pass in a path vector
    // if you want to draw the path.
    std::string drawMap(std::unordered_map<Point2d, double> *distances = nullptr,
                        std::unordered_map<Point2d, Point2d> *point_to = nullptr,
                        std::vector<Point2d> *path = nullptr,
                        Point2d *start = nullptr,
                        Point2d *goal = nullptr) const;
};
