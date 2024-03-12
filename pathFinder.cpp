#include "pathFinder.h"
#include "log.h"

std::variant<Path, PathfindingFailureReason> AStarPathfinder::findPath(const Point2d &start,
                                                                       const Point2d &goal,
                                                                       const Map &map)
{
    if (!map.inBounds(start) || !map.inBounds(goal))
        return PathfindingFailureReason::OUT_OF_BOUNDS;
    if (map.getCell(start) == MapItemSpace::MapItem::SEA ||
        map.getCell(start) == MapItemSpace::MapItem::OBSTACLE)
        return PathfindingFailureReason::START_POINT_INVALID;
    if (map.getCell(goal) == MapItemSpace::MapItem::SEA ||
        map.getCell(goal) == MapItemSpace::MapItem::OBSTACLE)
        return PathfindingFailureReason::END_POINT_INVALID;
    if (start == goal)
        return PathfindingFailureReason::START_AND_END_POINT_SAME;

    std::unordered_map<Point2d, Point2d> came_from;  // came_from 用于追踪路径
    std::unordered_map<Point2d, int> cost_so_far; // cost_so_far 用于记录到达每个点的成本
    aStarSearch(map, start, goal, came_from, cost_so_far);

    // 如果未找到路径（即目标不在 came_from 中）
    if (came_from.find(goal) == came_from.end())
    {
        return PathfindingFailureReason::NO_PATH_EXISTS;
    }

    // 回溯路径
    Path path = reconstruct_path(start, goal, came_from);
    return path;
}

template <typename Location, typename Graph>
void AStarPathfinder::aStarSearch(const Graph &graph,
                                  const Location &start,
                                  const Location &goal,
                                  std::unordered_map<Location, Location> &came_from,
                                  std::unordered_map<Location, int> &cost_so_far)
{
    // int calTime = 0;
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
        // calTime += 4;
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
    // LOGI("A* 遍历节点个数：",calTime);
    // LOGI("优先队列长度：",frontier.elements.size());
}

template <typename Location>
std::vector<Location> AStarPathfinder::reconstruct_path(
    const Location &start, const Location &goal,
    const std::unordered_map<Location, Location>& came_from)
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