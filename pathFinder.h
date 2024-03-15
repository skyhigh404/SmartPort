#pragma once
#include "map.h"
#include "utils.h"
#include "priorityQueue.h"
#include <vector>
#include <variant>
#include <algorithm>
#include <limits>

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
                     std::unordered_map<Location, int> &cost_so_far);

    template <typename Location>
    std::vector<Location> reconstruct_path(
        const Location &start, const Location &goal,
        const std::unordered_map<Location, Location> &came_from);

public:
    inline int heuristic(Point2d pos1, Point2d pos2)
    {
        return Point2d::calculateManhattanDistance(pos1, pos2);
    }
};

// struct Node
// {
//     Point2d pos;
//     float RHS; // 到终点的 cost
//     float G;   // 到终点的 cost
//     float H;   // 到起点的启发式 cost

//     using TLimits = std::numeric_limits<float>;
//     Node(Point2d pos, float rhs = TLimits::max(), float g = TLimits::max()) : pos(pos), RHS(rhs), G(g) {}
// };

class DStarPathfinder
{

private:
    struct Key
    {
        Cost first = 0.0;
        Cost second = 0.0;

        bool operator<(const Key &rhs) const
        {
            if (first < rhs.first)
            {
                return true;
            }
            else if (first == rhs.first)
            {
                return second < rhs.second;
            }
            else
            {
                return false;
            }
        }
        bool operator==(const Key &rhs) const
        {
            return first == rhs.first && second == rhs.second;
        }
        bool operator>(const Key &rhs) const
        {
            return !operator<(rhs) && !operator==(rhs);
        }
        bool operator>=(const Key &rhs) const
        {
            return operator>(rhs) || operator==(rhs);
        }
        bool operator<=(const Key &rhs) const
        {
            return operator<(rhs) || operator==(rhs);
        }
    };

    Point2d start, goal, last_start;

    PriorityQueueWithRemove<Point2d, Key> pq;
    HashMap<Point2d, Cost> gscores; // G
    HashMap<Point2d, Cost> rscores; // RHS
    Cost k = Cost(0);// 键值修饰器，用于修饰Key，改变Key的值


public:
    Path plan(const Point2d &start, const Point2d &goal, const Map &map);
    Path replan(const Map &map, const std::vector<Point2d> &changedStates);
    // 从一个新的起始点规划到原来的终点
    Path replan(const Point2d &start, const Map &map, const std::vector<Point2d> &changedStates);

private:
    void initialize();
    void updateVertex(const Point2d &pos);
    Key calculate_key(const Point2d &pos);
    // Node getState(const Node &current);
    Path computeShortesPath(const Map &map);

    Cost heuristic(const Point2d &from, const Point2d &to)
    {
        return Cost(Point2d::calculateManhattanDistance(from, to));
    }
    
    Path backtrack(const Map &map);
};