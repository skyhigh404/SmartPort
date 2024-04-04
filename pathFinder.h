#pragma once
#include "map.h"
#include "utils.h"
#include "priorityQueue.h"
#include <vector>
#include <variant>
#include <algorithm>
#include <limits>

template <typename Location>
using Path = std::vector<Location>;

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

template <class Location, class Graph>
class Pathfinder
{
    // 怎么去
public:
    // 寻路算法的输入是起始点和目标点的坐标，以及地图信息。输出是一系列坐标，表示路径。
    // 如果无法到达，返回 PathfindingFailureReason。
    virtual std::variant<Path<Location>, PathfindingFailureReason>
    findPath(const Location &start, const Location &goal, const Graph &graph) = 0;
};

template <class Location, class Graph>
class AStarPathfinder : public Pathfinder<Location, Graph>
{
public:
    // Path 第一个元素是终点，逆序存储
    virtual std::variant<Path<Location>, PathfindingFailureReason>
    findPath(const Location &start,
             const Location &goal,
             const Graph &graph) override;

    void aStarSearch(const Graph &graph,
                     const Location &start,
                     const Location &goal,
                     std::unordered_map<Location, Location> &came_from,
                     std::unordered_map<Location, int> &cost_so_far);

    Path<Location> reconstruct_path(
        const Location &start, const Location &goal,
        const std::unordered_map<Location, Location> &came_from);

public:
    inline int heuristic(Point2d pos1, Point2d pos2)
    {
        return Point2d::calculateManhattanDistance(pos1, pos2);
    }
};
