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
        SPACE = 0,      // 空地
        MAIN_ROAD,      // 陆地主干道
        SEA,            // 海洋
        SEA_LANE,       // 海洋主航道
        OBSTACLE,       // 障碍
        ROBOT_SHOP,     // 机器人购买地块，同时该地块也是主干道
        ROBOT,          // 机器人
        SHIP_SHOP,      // 船舶购买地块，同时该地块也是主航道
        SHIP,           // 船舶
        BERTH,          // 泊位，特殊的海陆立体交通地块，视为主干道和主航道
        MOORING_AREA,   // 靠泊区，可作海洋主航道
        HYBRID,         // 海陆立体交通地块
        HYBRID_LANE,    // 海陆立体交通地块，同时为主干道和主航道
        DELIVERY_POINT, // 交货点，相当于特殊靠泊区

        ERROR = -1,
    };

}

enum class MapFlag
{
    NORMAL, // 正常参数
    ERROR   // 默认值
};

extern MapFlag MAP_TYPE;

class Map
{
    // 地图坐标系原点在左上角，往下为 X 轴正方向，往右为 Y 轴正方向
public:
    static std::array<Point2d, 4> DIRS;
    int rows, cols;
    std::vector<std::vector<MapItemSpace::MapItem>> grid;                    // 地图
    std::vector<std::vector<MapItemSpace::MapItem>> readOnlyGrid;                // 地图的拷贝，只读
    std::unordered_map<int, std::vector<std::vector<int>>> berthDistanceMap; // 泊位距离图
public:
    std::vector<std::reference_wrapper<Point2d>> robotPosition;  // 实时记录机器人位置（不建议使用）
    std::vector<Point2d> temporaryObstacles;                     // 临时障碍物的位置
    std::unordered_map<Point2d, int> temporaryObstaclesRefCount; // 对障碍物进行计数

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

    // 查询 pos 位置是否可达
    inline bool passable(const Point2d &pos) const
    {
        MapItemSpace::MapItem item = getCell(pos);
        return (item != MapItemSpace::MapItem::OBSTACLE &&
                item != MapItemSpace::MapItem::SEA &&
                item != MapItemSpace::MapItem::ROBOT);
    }

    float costCosin(const Point2d &robotPos, const Point2d &goodPos, const Point2d &berthPos, const int berthID);

public:
    // 计算泊位到地图上所有点的距离，不可通行的记录为 INT_MAX
    void computeDistancesToBerthViaBFS(BerthID id, const std::vector<Point2d> &positions);
    // 获取当前帧地图的变化，即机器人的位置，将其视为障碍（除自己外），预测未来 n 帧是否有碰撞风险
    std::vector<Point2d> isCollisionRisk(int robotID, int framesAhead) const;
    // 添加一个临时障碍物，即机器人
    void addTemporaryObstacle(const Point2d &pos);
    // 移除一个临时障碍物
    void removeTemporaryObstacle(const Point2d &pos);
    // 移除所有临时障碍物
    void clearTemporaryObstacles();
    // 获取距离为 n 格内除 robotPos 外的其他机器人坐标
    std::vector<Point2d> getNearbyTemporaryObstacles(const Point2d &robotPos, int n) const;

public:
    // This outputs a grid. Pass in a distances map if you want to print
    // the distances, or pass in a point_to map if you want to print
    // arrows that point to the parent location, or pass in a path vector
    // if you want to draw the path.
    std::string drawMap(std::unordered_map<Point2d, double> *distances = nullptr,
                        std::unordered_map<Point2d, Point2d> *point_to = nullptr,
                        std::vector<Point2d> *path = nullptr,
                        Point2d *start = nullptr,
                        Point2d *goal = nullptr) const;
    // 给定 field_width 的字符宽度，打印二维数组
    static std::string drawMap(std::vector<std::vector<int>>, int field_width);

    // 给定一个坐标，输出距离最近的泊位 ID
    int getNearestBerthID(const Point2d &pos) const;
    // 给定一个坐标和泊位 ID，判断是否可达
    bool isBerthReachable(BerthID id, Point2d &position) const;
    // 给定一个坐标和泊位 ID，得到泊位距离
    int getDistanceToBerth(BerthID id, Point2d &position) const;
    // 计算一个点到所有泊位的距离，以降序输出，第一个是泊位 ID，第二个是距离，不包含不可达泊位
    std::vector<std::pair<int, int>> computePointToBerthsDistances(Point2d position) const;
    // 返回当前节点上下左右的四个可达的邻居
    std::vector<Point2d> neighbors(Point2d id) const;
    // 使用曼哈顿距离计算两个点之间的代价
    inline int cost(Point2d pos1, Point2d pos2) const
    {
        return Point2d::calculateManhattanDistance(pos1, pos2);
    }
};

std::string printVector(const std::vector<Point2d> &path);
