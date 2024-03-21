#pragma once

#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include <functional>
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
    std::unordered_map<int, std::vector<std::vector<int>>> berthDistanceMap;
    std::vector<std::reference_wrapper<Point2d>> robotPosition; // 实时记录机器人位置
    std::vector<Point2d> temporaryObstacles; // 临时障碍物的位置
    std::unordered_map<Point2d, int> temporaryObstaclesRefCount;    // 对障碍物进行计数

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

    inline bool inBounds(const int x, const int y) const
    {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }
    inline bool inBounds(const Point2d &pos) const
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

    float costCosin(const Point2d &robotPos, const Point2d &goodPos, const Point2d &berthPos, const int berthID)
    {
        int berth2good = berthDistanceMap.at(berthID)[goodPos.x][goodPos.y];
        int berth2robot = berthDistanceMap.at(berthID)[robotPos.x][robotPos.y];

        float cosin = Vec2f::cosineOf2Vec(Vec2f(berthPos, robotPos), Vec2f(berthPos, goodPos));
        int cost = static_cast<int>(std::sqrt(berth2good * berth2good + berth2robot * berth2robot - 2 * berth2good * berth2robot * cosin));
        return cost;
    }
public:
    std::vector<Point2d> neighbors(Point2d id) const; // 返回当前节点上下左右的四个邻居
    inline bool passable(const Point2d &pos) const
    {
        MapItemSpace::MapItem item = getCell(pos);
        return (item != MapItemSpace::MapItem::OBSTACLE &&
                item != MapItemSpace::MapItem::SEA &&
                item != MapItemSpace::MapItem::ROBOT);
    }
    // 判断是否会被机器人占据，如果是则返回 false
    inline bool dynamicPassable(const Point2d &pos) const{
        return false;
    }
    // This outputs a grid. Pass in a distances map if you want to print
    // the distances, or pass in a point_to map if you want to print
    // arrows that point to the parent location, or pass in a path vector
    // if you want to draw the path.
    std::string drawMap(std::unordered_map<Point2d, double> *distances = nullptr,
                        std::unordered_map<Point2d, Point2d> *point_to = nullptr,
                        std::vector<Point2d> *path = nullptr,
                        Point2d *start = nullptr,
                        Point2d *goal = nullptr) const;
    static std::string drawMap(std::vector<std::vector<int>>, int field_width);
    void computeDistancesToBerthViaBFS(BerthID id, const std::vector<Point2d> &positions);

    bool isBerthReachable(BerthID id, Point2d position);
    // 获取当前帧地图的变化，即机器人的位置，将其视为障碍（除自己外）。
    // 预测未来 n 帧是否有碰撞风险
    std::vector<Point2d> isCollisionRisk(int robotID, int framesAhead) const;
    // 添加临时障碍物，即机器人
    void addTemporaryObstacle(const Point2d& pos);
    void removeTemporaryObstacle(const Point2d& pos);
    void clearTemporaryObstacles();
    std::vector<Point2d> getNearbyTemporaryObstacles(const Point2d& robotPos, int n) const;
};

std::string printVector(const std::vector<Point2d> &path);
