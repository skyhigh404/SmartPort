#include "pathFinder.h"
#include "log.h"
#include <chrono>

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

    std::unordered_map<Point2d, Point2d> came_from; // came_from 用于追踪路径
    std::unordered_map<Point2d, int> cost_so_far;   // cost_so_far 用于记录到达每个点的成本
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

void DStarPathfinder::initialize()
{
    pq = PriorityQueueWithRemove<Point2d, Key>();
    gscores.clear();
    rscores.clear();

    rscores.put(goal, 0.0);
    pq.insert(goal, calculate_key(goal));
    k = Cost(0);
}

Path DStarPathfinder::plan(const Point2d &start, const Point2d &goal, const Map &map)
{
    // 初始化
    this->last_start = start;
    this->start = start;
    this->goal = goal;

    initialize();
    const auto path = computeShortesPath(map);
    return path;
}

Path DStarPathfinder::replan(const Map &map, const std::vector<Point2d> &changedStates, const std::vector<bool> &isObstacle)
{
    for (int i = 0; i < changedStates.size(); ++i)
    {
        const Point2d &u = changedStates[i];
        for (const auto &v : map.neighbors(u))
        {
            // 如果是可通行的，说明上一时刻是不可通行的；反之亦然
            // Cost oldCost = map.dynamicPassable(u) ? Cost::max() : heuristic(u, v);
            // Cost oldCost = isObstacle[i] ? heuristic(u, v) : Cost::max();
            Cost oldCost = heuristic(u, v);
            // Cost oldCost = map.dynamicPassable(u) ? heuristic(u, v) : Cost::max();
            if (oldCost > heuristic(u, v) && u != goal)
            {
                // 该位置变为可通行，C_old > C(u,v)，更新 rhs
                rscores.put(u, std::min(rscores.at(u), heuristic(u, v) + gscores.at(v)));
            }
            else if (rscores.at(u) == oldCost + gscores.at(v) && u != goal)
            {
                LOGI("新增障碍物");
                // 该位置是新增障碍物，C_old < C(u,v)，仅当旧路径通过当前节点时更新 rhs
                // min_{s\in pred(u)}(g(s) + c(s, u))
                Cost minCost = Cost::max();
                // for (const auto &succ : map.neighbors(u))
                // {
                //     minCost = std::min(minCost, heuristic(u, succ) + gscores.at(succ));
                // }
                rscores.put(u, minCost);
            }
        }
        updateVertex(u);
    }

    const auto path = computeShortesPath(map);

    return path;
}

Path DStarPathfinder::replan(const Point2d &start, const Map &map, const std::vector<Point2d> &changedStates, const std::vector<bool> &isObstacle)
{
    if (this->start == start)
    {
        return replan(map, changedStates, isObstacle);
    }
    else
    {
        this->last_start = this->start;
        this->start = start;
        k = k + heuristic(this->last_start, this->start);
        return replan(map, changedStates, isObstacle);
    }
}



Path DStarPathfinder::computeShortesPath(const Map &map)
{
    auto startTime = std::chrono::steady_clock::now();
    bool isSuccess = true;
    while (true)
    {

        if (rscores.at(start) <= gscores.at(start) &&
            gscores.at(start) < Cost::max())
        {
            if (pq.empty() || pq.top_priority() >= calculate_key(start))
            {
                break;
            }
        }

        if (pq.empty())
        {
            isSuccess = false;
            break;
        }

        const auto curr_state = pq.top();
        const auto key_old = pq.top_priority();
        const auto key_new = calculate_key(curr_state);

        if (key_old < key_new)
        {
            pq.insert(curr_state, key_new);
        }
        else if (gscores.at(curr_state) > rscores.at(curr_state) ||
                 gscores.at(curr_state) == Cost::max())
        {
            // 局部欠一致，障碍物被移除，发现了更好的点
            gscores.put(curr_state, rscores.at(curr_state)); // 更新 G 值
            pq.remove(curr_state);
            for (const auto &pred : map.neighbors(curr_state)) // get_predecessors 更新所有前继节点的 rhs 并重新入队（当最优路径更新时）
            {
                if (!(pred == goal))
                {
                    const auto r = rscores.at(pred);
                    const auto c = heuristic(pred, curr_state);
                    const auto g = gscores.at(curr_state);
                    rscores.put(pred, std::min(r, c + g));
                }
                updateVertex(pred);
            }
        }
        else
        {
            // rhs > g 局部过一致，障碍物加入
            // LOGI("加入障碍物");
            auto g_old = gscores.at(curr_state);
            gscores.put(curr_state, Cost::max());
            auto nodes = map.neighbors(curr_state); // get_predecessors
            nodes.push_back(curr_state);
            for (const auto &s : nodes) // 对该节点以及对应的所有前继节点，更新所有后继节点的值
            {
                if (rscores.at(s) == heuristic(s, curr_state) + g_old)
                {
                    if (!(s == goal))
                    {
                        auto min_c = Cost::max();
                        for (const auto &sp : map.neighbors(s)) // get_successors
                        {
                            min_c = std::min(min_c, heuristic(s, sp) + gscores.at(sp));
                        }
                        rscores.put(s, min_c);
                    }
                }
                updateVertex(s);
            }
        }

        if(std::chrono::steady_clock::now() - startTime > std::chrono::milliseconds(1000)){
            LOGW("寻路超时 start: ", start, " goal: ", goal, " queue size", pq.size());
            isSuccess = false;
            break;
        }

        
    }

    if (isSuccess)
    {
        return backtrack(map);
    }
    else
    {
        // LOGW("fail");
        return {};
    }
}

void DStarPathfinder::updateVertex(const Point2d &pos)
{
    pq.remove(pos);
    if (gscores.at(pos) != rscores.at(pos) ||
        gscores.at(pos) == Cost::max())
    {
        pq.insert(pos, calculate_key(pos));
    }
}
DStarPathfinder::Key DStarPathfinder::calculate_key(const Point2d &pos)
{
    Cost g = gscores.at(pos);
    Cost rhs = rscores.at(pos);
    Cost h = heuristic(pos, start);

    Key key;
    // key.first = std::min(g, rhs + h + k); // ? 这里没问题吗
    key.first = std::min(g, rhs) + h + k; // 这是我认为的版本
    key.second = std::min(g, rhs);
    return key;
}

Path DStarPathfinder::backtrack(const Map &map)
{
    int count = 0;
    Path path;

    Point2d state = start;
    while (state != goal)
    {
        const auto &succs = map.neighbors(state); // get_successors

        // Find the cheapest predecessor to this state.
        Point2d best_succ = succs.at(0);
        {
            Cost min_cost = Cost::max();
            for (const auto &succ : succs)
            {
                Cost new_cost = gscores.at(succ) + heuristic(state, succ);
                if (new_cost < min_cost)
                {
                    min_cost = new_cost;
                    best_succ = succ;
                }
            }
        }
        path.push_back(state);
        state = best_succ;
        if (count++ > 1000)
        {
            LOGW("backtrack 太多次, 路径不可达 ", "start: ", start, " goal: ", goal);
            return {};
        }
    }
    // 这里的路径包含起始点，删除掉
    path.push_back(goal);
    std::reverse(path.begin(), path.end());
    path.pop_back();

    return path;
}
