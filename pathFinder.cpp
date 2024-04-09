#include "pathFinder.h"
#include "log.h"
#include <chrono>

template <class Location, class Graph>
std::variant<Path<Location>, PathfindingFailureReason>
AStarPathfinder<Location, Graph>::findPath(const Location &start,
                                           const Location &goal,
                                           const Graph &graph)
{
    if (!graph.inBounds(start) || !graph.inBounds(goal))
        return PathfindingFailureReason::OUT_OF_BOUNDS;
    if (!graph.passable(start))
        return PathfindingFailureReason::START_POINT_INVALID;
    if (!graph.passable(goal))
        return PathfindingFailureReason::END_POINT_INVALID;
    if (start == goal)
        return PathfindingFailureReason::START_AND_END_POINT_SAME;

    std::unordered_map<Location, Location> came_from; // came_from 用于追踪路径
    std::unordered_map<Location, int> cost_so_far;    // cost_so_far 用于记录到达每个点的成本
    aStarSearch(graph, start, goal, came_from, cost_so_far);

    // 如果未找到路径（即目标不在 came_from 中）
    if (came_from.find(goal) == came_from.end())
    {
        return PathfindingFailureReason::NO_PATH_EXISTS;
    }

    // 回溯路径
    Path<Location> path = reconstruct_path(start, goal, came_from);
    return path;
}

template <class Location, class Graph>
void AStarPathfinder<Location, Graph>::aStarSearch(const Graph &graph,
                                                   const Location &start,
                                                   const Location &goal,
                                                   std::unordered_map<Location, Location> &came_from,
                                                   std::unordered_map<Location, int> &cost_so_far)
{
    int calTime = 0;
    if (!graph.inBounds(goal) || !graph.passable(goal))
        return;
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty())
    {
        const Location &current = frontier.get();

        if (current == goal)
        {
            break;
        }
#ifdef DEBUG
        calTime += 4;
#endif
        for (const Location &next : graph.neighbors(current))
        {
            int new_cost = cost_so_far[current] + graph.cost(current, next);
            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next])
            {
                cost_so_far[next] = new_cost;
                int priority = new_cost + heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }
    // LOGI("A* 搜索节点个数：", calTime);
    // LOGI("优先队列长度：",frontier.elements.size());
}

template <class Location, class Graph>
Path<Location> AStarPathfinder<Location, Graph>::reconstruct_path(
    const Location &start, const Location &goal,
    const std::unordered_map<Location, Location> &came_from)
{
    std::vector<Location> path;
    Location current = goal;
    if (came_from.find(goal) == came_from.end())
    {
        return path; // no path can be found
    }
    while (current != start)
    {
        path.push_back(current);
        current = came_from.at(current);
    }
    // path.push_back(start); // optional
    // std::reverse(path.begin(), path.end());
    // LOGI("路径长度：",path.size());
    return path;
}

// 显式实例化
template class AStarPathfinder<VectorPosition, Map>;
template class AStarPathfinder<Point2d, Map>;