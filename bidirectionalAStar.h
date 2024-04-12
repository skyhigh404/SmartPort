#include "pathFinder.h"
#include <mutex>
#include <optional>
#include <vector>
#include <future>
#include <atomic>

template <class Location, class Graph>
class BidirectionalAStarPathfinder : public Pathfinder<Location, Graph>
{
private:
    std::unordered_map<Location, Location> came_from_start, came_from_goal; // came_from 用于追踪路径
    std::unordered_map<Location, int> cost_so_far_start, cost_so_far_goal;  // cost_so_far 用于记录到达每个点的成本
    PriorityQueue<Location, double> frontier_start, frontier_goal;
    std::mutex search_from_goal_mutex, search_from_start_mutex;
    std::optional<Location> meeting_point;
    std::atomic<bool> found_meeting_point{false};

public:
    // Path 第一个元素是终点，逆序存储
    virtual std::variant<Path<Location>, PathfindingFailureReason>
    findPath(const Location &start,
             const Location &goal,
             const Graph &graph) override
    {
        if (!graph.inBounds(start) || !graph.inBounds(goal))
            return PathfindingFailureReason::OUT_OF_BOUNDS;
        if (!graph.passable(start))
            return PathfindingFailureReason::START_POINT_INVALID;
        if (!graph.passable(goal))
            return PathfindingFailureReason::END_POINT_INVALID;
        if (start == goal)
            return PathfindingFailureReason::START_AND_END_POINT_SAME;
        
        auto search_from_start = std::async(std::launch::async, [&]
                                            { searchFromStart(graph, start); });
        auto search_from_goal = std::async(std::launch::async, [&]
                                           { searchFromGoal(graph, goal); });

        // 等待两个方向的搜索完成
        search_from_start.wait();
        search_from_goal.wait();

        // 如果未找到路径
        if (!meeting_point.has_value())
        {
            return PathfindingFailureReason::NO_PATH_EXISTS;
        }

        // 回溯路径
        Path<Location> path = reconstruct_path(start, goal);
        return path;
    }

    void searchFromStart(const Graph &graph, const Location &start)
    {
        frontier_start.put(start, 0);
        came_from_start[start] = start;
        cost_so_far_start[start] = 0;

        while (!frontier_start.empty())
        {
            Location current = frontier_start.get();

            // 检查是否遇到了从另一侧开始的搜索
            {
                // std::lock_guard<std::mutex> lock(search_from_goal_mutex);
                if (came_from_goal.find(current) != came_from_goal.end())
                {
                    meeting_point = current;         // 记录相遇点
                    found_meeting_point.store(true); // 通知其他搜索方向
                    break;
                }
            }

            for (const Location &next : graph.neighbors(current))
            {
                if (!graph.passable(next))
                    continue; // 忽略不可通过的节点

                int new_cost = cost_so_far_start[current] + graph.cost(current, next);
                if (!cost_so_far_start.count(next) || new_cost < cost_so_far_start[next])
                {
                    cost_so_far_start[next] = new_cost;
                    int priority = new_cost + heuristic(next, start); // 注意：启发式函数的目标位置取决于搜索方向
                    frontier_start.put(next, priority);
                    // std::lock_guard<std::mutex> lock(search_from_start_mutex);
                    came_from_start[next] = current;
                }
            }
        }
    }

    void searchFromGoal(const Graph &graph, const Location &goal)
    {
        frontier_goal.put(goal, 0);
        came_from_goal[goal] = goal;
        cost_so_far_goal[goal] = 0;

        while (!frontier_goal.empty())
        {
            Location current = frontier_goal.get();

            // 检查是否遇到了从另一侧开始的搜索
            {
                // std::lock_guard<std::mutex> lock(search_from_start_mutex);
                if (came_from_start.find(current) != came_from_start.end())
                {
                    meeting_point = current;         // 找到相遇点
                    found_meeting_point.store(true); // 通知其他搜索方向
                    break;
                }
            }

            for (const Location &next : graph.neighbors(current))
            {
                if (!graph.passable(next))
                    continue; // 忽略不可通过的节点

                int new_cost = cost_so_far_goal[current] + graph.cost(current, next);
                if (!cost_so_far_goal.count(next) || new_cost < cost_so_far_goal[next])
                {
                    cost_so_far_goal[next] = new_cost;
                    int priority = new_cost + heuristic(next, goal); // 注意：启发式函数的目标位置取决于搜索方向
                    frontier_goal.put(next, priority);
                    // std::lock_guard<std::mutex> lock(search_from_goal_mutex);
                    came_from_goal[next] = current;
                }
            }
        }
    }

    Path<Location> reconstruct_path(const Location &start, const Location &goal)
    {
        if (!meeting_point.has_value())
        {
            return Path<Location>(); // 如果没有相遇点，直接返回空路径
        }

        Location meet = meeting_point.value();
        std::vector<Location> path;
        std::vector<Location> path_to_meet;
        std::vector<Location> path_from_meet;

        // 从相遇点回溯到起点
        Location current = meet;
        while (current != start)
        {
            path_to_meet.push_back(current);
            current = came_from_start.at(current);
        }
        path_to_meet.push_back(start);                          // 加入起点
        std::reverse(path_to_meet.begin(), path_to_meet.end()); // 反转，因为我们是从相遇点回溯到起点

        // 从相遇点回溯到终点
        current = meet;
        while (current != goal)
        {
            path_from_meet.push_back(current);
            current = came_from_goal.at(current);
        }
        path_from_meet.push_back(goal); // 加入终点

        // 合并两个路径段
        path.reserve(path_to_meet.size() + path_from_meet.size() - 1); // 相遇点会被重复加入，因此-1
        path.insert(path.end(), path_to_meet.begin(), path_to_meet.end());
        path.insert(path.end(), path_from_meet.begin() + 1, path_from_meet.end()); // 从第二个元素开始加入，避免重复相遇点

        return path;
    }

    inline int heuristic(const Point2d &pos1, const Point2d &pos2)
    {
        return Point2d::calculateManhattanDistance(pos1, pos2);
    }
};
