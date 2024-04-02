#include "map.h"
#include <algorithm>
#include <sstream>
#include <string>
#include <iomanip>
#include <queue>
#include <climits>
#include "log.h"

std::array<Point2d, 4> Map::DIRS = {
    /* East, West, North, South */
    Point2d{1, 0}, Point2d{-1, 0}, Point2d{0, -1}, Point2d{0, 1}};

std::vector<Point2d> Map::neighbors(Point2d pos) const
{
    std::vector<Point2d> results;
    results.reserve(4);

    for (const Point2d &dir : DIRS)
    {
        Point2d next{pos.x + dir.x, pos.y + dir.y};
        if (inBounds(next) && passable(next))
        {
            results.push_back(next);
        }
    }

    if ((pos.x + pos.y) % 2 == 0)
    {
        // see "Ugly paths" section for an explanation:
        std::reverse(results.begin(), results.end());
    }

    return results;
}

bool Map::isInMainRodad(const Point2d &pos) const
{
    MapItemSpace::MapItem item = getCell(pos);
    return (item == MapItemSpace::MapItem::MAIN_ROAD ||
            item == MapItemSpace::MapItem::ROBOT_SHOP ||
            item == MapItemSpace::MapItem::BERTH ||
            item == MapItemSpace::MapItem::HYBRID_LANE);
}

bool Map::isInSealane(const Point2d &pos) const
{
    MapItemSpace::MapItem item = getCell(pos);
    return (item == MapItemSpace::MapItem::SEA_LANE ||
            item == MapItemSpace::MapItem::SHIP_SHOP ||
            item == MapItemSpace::MapItem::BERTH ||
            item == MapItemSpace::MapItem::MOORING_AREA ||
            item == MapItemSpace::MapItem::HYBRID_LANE ||
            item == MapItemSpace::MapItem::DELIVERY_POINT);
}

std::string Map::drawMap(std::unordered_map<Point2d, double> *distances,
                         std::unordered_map<Point2d, Point2d> *point_to,
                         std::vector<Point2d> *path,
                         Point2d *start,
                         Point2d *goal) const
{
    const int field_width = 3;
    using namespace MapItemSpace;
    using std::string, std::find;
    std::ostringstream oss;

    oss << std::string(field_width * cols, '_') << '\n';
    for (int x = 0; x < rows; ++x)
    {
        for (int y = 0; y < cols; ++y)
        {
            Point2d pos{x, y};
            MapItem item = getCell(pos);

            if (start && pos == *start)
                oss << " A ";
            else if (goal && pos == *goal)
                oss << " Z ";
            else if (path != nullptr && find(path->begin(), path->end(), pos) != path->end())
                oss << " @ ";
            else if (point_to != nullptr && point_to->count(pos))
            {
                Point2d next = (*point_to)[pos];
                if (next.x == x + 1)
                    oss << " > ";
                else if (next.x == x - 1)
                    oss << " < ";
                else if (next.y == y + 1)
                    oss << " v ";
                else if (next.y == y - 1)
                    oss << " ^ ";
                else
                    oss << " % ";
            }
            else if (distances != nullptr && distances->count(pos))
                oss << std::setw(field_width) << (*distances)[pos];
            else if (item == MapItem::OBSTACLE)
                oss << string(field_width, '#');
            else if (item == MapItem::SEA)
                oss << string(field_width, '*');
            else if (item == MapItem::BERTH)
                oss << " B ";
            else if (item == MapItem::SPACE)
                oss << " . ";
            else
                oss << " E ";
        }
        oss << "\n";
    }
    oss << std::string(field_width * cols, '~') << "\n";
    std::string result = oss.str();
    return result;
}

void Map::computeDistancesToBerthViaBFS(BerthID id, const std::vector<Point2d> &positions)
{
    using std::vector, std::queue;
    vector<vector<int>> dis(rows, vector<int>(cols, INT_MAX));
    queue<Point2d> nextToVisitQueue;
    for (const Point2d &pos : positions)
    {
        if (inBounds(pos) && passable(pos))
        {
            dis[pos.x][pos.y] = 0;
            nextToVisitQueue.push(pos);
        }
    }
    while (!nextToVisitQueue.empty())
    {
        Point2d current = nextToVisitQueue.front();
        nextToVisitQueue.pop();
        for (const Point2d &dir : DIRS)
        {
            Point2d next{current.x + dir.x, current.y + dir.y};
            if (inBounds(next) && passable(next) && dis[next.x][next.y] == INT_MAX)
            {
                dis[next.x][next.y] = dis[current.x][current.y] + 1;
                nextToVisitQueue.push(next);
            }
        }
    }
    berthDistanceMap[id] = dis;
}

std::string Map::drawMap(std::vector<std::vector<int>> map, int field_width)
{
    using std::string, std::find;
    std::ostringstream oss;
    int row = map.size(), col = map[0].size();

    oss << std::string(field_width * col, '_') << '\n';
    for (int x = 0; x < row; ++x)
    {
        for (int y = 0; y < col; ++y)
        {
            oss << std::setw(field_width) << map[x][y];
        }
        oss << "\n";
    }
    oss << std::string(field_width * col, '~') << "\n";
    std::string result = oss.str();
    return result;
}

bool Map::isBerthReachable(BerthID id, Point2d &position) const
{
    if (berthDistanceMap.at(id)[position.x][position.y] != INT_MAX)
        return true;
    return false;
}

int Map::getDistanceToBerth(BerthID id, Point2d &position) const
{
    return berthDistanceMap.at(id)[position.x][position.y];
}


std::vector<Point2d> Map::isCollisionRisk(int robotID, int framesAhead) const
{
    std::vector<Point2d> obstacle;
    obstacle.reserve(5 * framesAhead);
    for (int i = 0; i < robotPosition.size(); ++i)
    {
        if (i == robotID)
            continue; // 不考虑自身
        if (Point2d::calculateManhattanDistance(robotPosition[robotID], robotPosition[i]) <= 2 * framesAhead)
        {
            for (int j = -framesAhead; j <= framesAhead; ++j)
            {
                for (int k = -framesAhead; k <= framesAhead; ++k)
                {
                    Point2d next = Point2d(robotPosition[i].get().x + j, robotPosition[i].get().y + k);
                    if (inBounds(next) && passable(next))
                        obstacle.push_back(next);
                }
            }
        }
    }
    return obstacle;
}

void Map::addTemporaryObstacle(const Point2d& pos) {
    if (inBounds(pos)) {
        MapItemSpace::MapItem item = getCell(pos);
        if(item == MapItemSpace::MapItem::OBSTACLE || item == MapItemSpace::MapItem::SEA){
            LOGE("往障碍位置上放置临时障碍, pos: ", pos);
            return;
        }
        grid[pos.x][pos.y] = MapItemSpace::MapItem::ROBOT; // 标记为障碍物
        temporaryObstacles.push_back(pos); // 添加到临时障碍物列表
        temporaryObstaclesRefCount[pos]++;
    }
}

void Map::removeTemporaryObstacle(const Point2d& pos) {
    if (inBounds(pos)) {
        auto it = temporaryObstaclesRefCount.find(pos);
        if (it != temporaryObstaclesRefCount.end()) {
            if (--it->second <= 0) {
                temporaryObstaclesRefCount.erase(it);
                grid[pos.x][pos.y] = MapItemSpace::MapItem::SPACE;  // 恢复为空地
            }
        }
    }
}

void Map::clearTemporaryObstacles() {
    for (const Point2d& pos : temporaryObstacles) {
        // 在清除前检查该位置是否确实是OBSTACLE，以防误清
        if (grid[pos.x][pos.y] == MapItemSpace::MapItem::ROBOT) {
            grid[pos.x][pos.y] = MapItemSpace::MapItem::SPACE; // 恢复为空地
        }
    }
    temporaryObstacles.clear(); // 清空临时障碍物列表
    temporaryObstaclesRefCount.clear();
}

std::vector<Point2d> Map::getNearbyTemporaryObstacles(const Point2d& robotPos, int n) const {
    std::vector<Point2d> nearbyObstacles;
    for (int j = -n; j <= n; ++j){
        for (int k = -n; k <= n; ++k){
            Point2d next(robotPos + Point2d(j,k));
            if (inBounds(next) && next!=robotPos && getCell(next) == MapItemSpace::MapItem::ROBOT)
                nearbyObstacles.push_back(next);
        }
    }
    return nearbyObstacles;
}

int Map::getNearestBerthID(const Point2d& pos) const
{
    int result = -1;
    int distance = INT_MAX;
    if(inBounds(pos)) {
        for (const auto &[ID, map] : berthDistanceMap) {
            if (map.at(pos.x).at(pos.y) < distance) {
                distance = map.at(pos.x).at(pos.y);
                result = ID;
            }
        }
    }
    return result;
}

std::vector<std::pair<int, int>> Map::computePointToBerthsDistances(Point2d position) const
{
    std::vector<std::pair<int, int>> result;
    for (const auto& [berthID, map] : berthDistanceMap) {
        int dist = berthDistanceMap.at(berthID)[position.x][position.y];
        if (dist != INT_MAX)
            result.emplace_back(berthID, dist);
    }

    // 以升序排序距离
    std::sort(result.begin(), result.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return a.second < b.second;
    });
    return result;
}

float Map::costCosin(const Point2d &robotPos, const Point2d &goodPos, const Point2d &berthPos, const int berthID)
{
    int berth2good = berthDistanceMap.at(berthID)[goodPos.x][goodPos.y];
    int berth2robot = berthDistanceMap.at(berthID)[robotPos.x][robotPos.y];

    float cosin = Vec2f::cosineOf2Vec(Vec2f(berthPos, robotPos), Vec2f(berthPos, goodPos));
    int cost = static_cast<int>(std::sqrt(berth2good * berth2good + berth2robot * berth2robot - 2 * berth2good * berth2robot * cosin));
    return cost;
}

std::string printVector(const std::vector<Point2d> &path)
{
    std::ostringstream oss;
    for (const auto &val : path)
        oss << val << " ";
    return oss.str();
}