#pragma once
#include "robot.h"
#include "map.h"
#include "utils.h"
#include <vector>
#include <unordered_map>
#include <variant>
#include <algorithm>
#include <queue>

using Path = std::vector<Point2d>;
enum class PathfindingFailureReason
{
    START_POINT_INVALID = 0,  // 起点无效（位于不可通行区域，如障碍物）
    END_POINT_INVALID,        // 终点无效
    START_AND_END_POINT_SAME, // 起点和终点相同
    PATH_BLOCKED,             // 路径被阻挡
    NO_PATH_EXISTS,           // 不存在路径（被障碍物阻挡）
    OUT_OF_BOUNDS,            // 起点或终点超出搜索范围
    OTHER                     // 其他原因
};

class Pathfinder
{
    // 怎么去
public:
    // 寻路算法的输入是起始点和目标点的坐标，以及地图信息。输出是一系列坐标，表示路径。
    // 如果无法到达，返回 PathfindingFailureReason。
    virtual std::variant<Path, PathfindingFailureReason> findPath(const Point2d &start, const Point2d &goal, const Map &map) = 0;
};


template <typename T, typename priority_t>
struct PriorityQueue
{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

    inline bool empty() const
    {
        return elements.empty();
    }

    inline void put(T item, priority_t priority)
    {
        elements.emplace(priority, item);
    }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

class AStarPathfinder : public Pathfinder
{
public:
    // Path 第一个元素是终点，逆序存储
    std::variant<Path, PathfindingFailureReason> findPath(const Point2d &start, const Point2d &goal, const Map &map) override;

    template <typename Location, typename Graph>
    void aStarSearch(const Graph &graph,
                     const Location &start,
                     const Location &goal,
                     std::unordered_map<Location, Location> &came_from,
                     std::unordered_map<Location, double> &cost_so_far);

    template <typename Location>
    std::vector<Location> reconstruct_path(
        Location start, Location goal,
        std::unordered_map<Location, Location> came_from);

public:
    inline int heuristic(Point2d pos1, Point2d pos2){
        return Point2d::calculateManhattanDistance(pos1, pos2);
    }
};
