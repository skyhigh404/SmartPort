#include "map.h"
#include <algorithm>
#include <sstream>
#include <string>
#include <iomanip>
#include <queue>
#include <climits>
#include "log.h"
#include <unordered_set>
#include <cstdlib>


std::array<Point2d, 4> Map::DIRS = {
    /* East, West, North, South */
    Point2d{1, 0}, Point2d{-1, 0}, Point2d{0, -1}, Point2d{0, 1}};

std::vector<Point2d> Map::neighbors(const Point2d &pos) const
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

    if ((pos.x + pos.y) % (rand()%3+1) == 0)
    {
        // see "Ugly paths" section for an explanation:
        std::reverse(results.begin(), results.end());
    }

    return results;
}

std::vector<VectorPosition> Map::neighbors(const VectorPosition &vp) const
{
    std::vector<VectorPosition> results;
    results.reserve(3);
    // 前进 1 格或者旋转一次
    // TODO: 某些区域进行了多次检查，可以进行优化
    VectorPosition moveForward = SpatialUtils::moveForward(vp);
    if(inBounds(moveForward) && passable(moveForward))
        results.push_back(moveForward);
    VectorPosition rotate = SpatialUtils::anticlockwiseRotation(vp);
    if(inBounds(rotate) && passable(rotate))
        results.push_back(rotate);
    rotate = SpatialUtils::clockwiseRotation(vp);
    if(inBounds(rotate) && passable(rotate))
        results.push_back(rotate);
    return results;
}

bool Map::isInMainRoad(const Point2d &pos) const
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

bool Map::inBounds(const VectorPosition &vp) const
{
    auto [topLeft, bottomRight] = SpatialUtils::getShipOccupancyRect(vp);
    for (int x = topLeft.x; x <= bottomRight.x; ++x)
    {
        for (int y = topLeft.y; y <= bottomRight.y; ++y)
        {
            if (!inBounds(Point2d(x, y)))
            {
                return false;
            }
        }
    }
    return true;
}

bool Map::passable(const VectorPosition &vp) const
{
    auto [topLeft, bottomRight] = SpatialUtils::getShipOccupancyRect(vp);
    for (int x = topLeft.x; x <= bottomRight.x; ++x)
    {
        for (int y = topLeft.y; y <= bottomRight.y; ++y)
        {
            if (!seaPassable(Point2d(x, y)))
            {
                return false;
            }
        }
    }
    return true;
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
            else if (item == MapItem::DELIVERY_POINT)
                oss << " T ";
            else if (item == MapItem::SEA_LANE)
                oss << " ~ ";
            else if (item == MapItem::HYBRID)
                oss << " C ";
            else if (item == MapItem::HYBRID_LANE)
                oss << " c ";
            else if (item == MapItem::MAIN_ROAD)
                oss << " = ";
            else if (item == MapItem::MOORING_AREA)
                oss << " K ";
            else if (item == MapItem::ROBOT_SHOP)
                oss << " R ";
            else if (item == MapItem::SHIP_SHOP)
                oss << " S ";
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

void Map::computeMaritimeBerthDistanceViaBFS(BerthID id, const std::vector<Point2d> &positions)
{
    using std::vector, std::queue;
    vector<vector<int>> dis(rows, vector<int>(cols, INT_MAX));
    queue<Point2d> nextToVisitQueue;
    for (const Point2d &pos : positions)
    {
        if (inBounds(pos) && seaPassable(pos))
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
            if (inBounds(next) && seaPassable(next) && dis[next.x][next.y] == INT_MAX)
            {
                if ((inBounds(VectorPosition(next, Direction::EAST)) && passable(VectorPosition(next, Direction::EAST))) ||
                    (inBounds(VectorPosition(next, Direction::WEST)) && passable(VectorPosition(next, Direction::WEST))) ||
                    (inBounds(VectorPosition(next, Direction::NORTH)) && passable(VectorPosition(next, Direction::NORTH))) ||
                    (inBounds(VectorPosition(next, Direction::SOUTH)) && passable(VectorPosition(next, Direction::SOUTH))))
                {
                    dis[next.x][next.y] = dis[current.x][current.y] + 1;
                    nextToVisitQueue.push(next);
                }
            }
        }
    }
    maritimeBerthDistanceMap[id] = dis;
}

Direction Map::computeBerthOrientation(const Point2d &pos)
{
    // TODO: 这里只假设给的点是泊位左上角，比较粗糙
    bool flag = false;
    // 检查是否朝右
    for(int i = 0; i < 2; ++i) {
        for(int j = 0; j < 3; ++j) {
            Point2d tmp = pos + Point2d(i, j);
            // LOGI("check ", tmp, ", ", static_cast<int>(getCell(tmp)));
            // 不是朝右的，那就是朝下的
            if (!inBounds(tmp) || getCell(tmp) != MapItemSpace::MapItem::BERTH){
                flag = true;
                // return Direction::SOUTH;
            }
        } 
    }
    if(!flag)
        return Direction::EAST;

    // 检查是否朝上
    flag = false;
    for(int i = -2; i <= 0; ++i) {
        for(int j = 0; j < 2; ++j) {
            Point2d tmp = pos + Point2d(i, j);
            // 不是朝右的，那就是朝下的
            if (!inBounds(tmp) || getCell(tmp) != MapItemSpace::MapItem::BERTH){
                flag = true;
            }
        } 
    }
    if(!flag)
        return Direction::NORTH;

    // 检查是否朝左
    flag = false;
    for(int i = -1; i <= 0; ++i) {
        for(int j = -2; j <= 0; ++j) {
            Point2d tmp = pos + Point2d(i, j);
            // 不是朝右的，那就是朝下的
            if (!inBounds(tmp) || getCell(tmp) != MapItemSpace::MapItem::BERTH){
                flag = true;
            }
        } 
    }
    if(!flag)
        return Direction::WEST;
    
    // 检查是否朝下
    flag = false;
    for(int i = 0; i < 3; ++i) {
        for(int j = -1; j <= 0; ++j) {
            Point2d tmp = pos + Point2d(i, j);
            // 不是朝右的，那就是朝下的
            if (!inBounds(tmp) || getCell(tmp) != MapItemSpace::MapItem::BERTH){
                flag = true;
            }
        } 
    }
    if(!flag)
        return Direction::SOUTH;
    
    LOGE("检查pos: ", pos, " 朝向失败");
    return Direction::EAST;
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


// std::vector<Point2d> Map::isCollisionRisk(int robotID, int framesAhead) const
// {
//     std::vector<Point2d> obstacle;
//     obstacle.reserve(5 * framesAhead);
//     for (int i = 0; i < robotPosition.size(); ++i)
//     {
//         if (i == robotID)
//             continue; // 不考虑自身
//         if (Point2d::calculateManhattanDistance(robotPosition[robotID], robotPosition[i]) <= 2 * framesAhead)
//         {
//             for (int j = -framesAhead; j <= framesAhead; ++j)
//             {
//                 for (int k = -framesAhead; k <= framesAhead; ++k)
//                 {
//                     Point2d next = Point2d(robotPosition[i].get().x + j, robotPosition[i].get().y + k);
//                     if (inBounds(next) && passable(next))
//                         obstacle.push_back(next);
//                 }
//             }
//         }
//     }
//     return obstacle;
// }

void Map::addTemporaryObstacle(const Point2d& pos) {
    if (inBounds(pos)) {
        MapItemSpace::MapItem item = getCell(pos);
        if(item == MapItemSpace::MapItem::OBSTACLE || item == MapItemSpace::MapItem::SEA){
            LOGE("往障碍位置上放置临时障碍, pos: ", pos);
            return;
        }
        else if(isInMainRoad(pos)){
            // LOGE("往主干道上放置临时障碍, pos: ", pos);
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
                grid[pos.x][pos.y] = readOnlyGrid[pos.x][pos.y];  // 恢复为原始元素
            }
        }
    }
}

void Map::addTemporaryObstacle(const VectorPosition& vecPos) {
    // 获取船的占用体积
    std::pair<Point2d, Point2d> shipSpace = SpatialUtils::getShipOccupancyRect(vecPos);
    for (int x = shipSpace.first.x; x <= shipSpace.second.x; x++){
        for (int y= shipSpace.first.y; y <= shipSpace.second.y; y++) {
            Point2d pos(x, y);
            if (inBounds(pos))
            {
                MapItemSpace::MapItem item = getCell(pos);
                if (item == MapItemSpace::MapItem::OBSTACLE || item == MapItemSpace::MapItem::SPACE)
                {
                    LOGE("往船舶不可通行位置上放置临时障碍, pos: ", pos);
                    continue;
                }
                else if (isInSealane(pos))
                {
                    // LOGE("往海洋主干道上放置临时障碍, pos: ", pos);
                    continue;
                }
                grid[pos.x][pos.y] = MapItemSpace::MapItem::SHIP; // 标记为障碍物
                temporaryObstacles.push_back(pos);                 // 添加到临时障碍物列表
                temporaryObstaclesRefCount[pos]++;
            }
        }
    }
}


void Map::removeTemporaryObstacle(const VectorPosition& vecPos) {
    // 获取船的占用体积
    std::pair<Point2d, Point2d> shipSpace = SpatialUtils::getShipOccupancyRect(vecPos);
    for (int x = shipSpace.first.x; x <= shipSpace.second.x; x++){
        for (int y= shipSpace.first.y; y <= shipSpace.second.y; y++)
            removeTemporaryObstacle({x, y});
    }
}

void Map::clearTemporaryObstacles() {
    for (const Point2d& pos : temporaryObstacles) {
        grid[pos.x][pos.y] = readOnlyGrid[pos.x][pos.y];  // 恢复为原始元素
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

int Map::cost(const VectorPosition &e1, const VectorPosition &e2) const
{
    // 邻居节点的代价为 1，处在主航道则为 2
    // TODO: 下面的计算方法不能处理旋转后核心点的变化问题，会增加旋转的代价
    // int result = cost(e1.pos, e2.pos) + abs(VectorPosition::minimalRotationStep(e1.direction, e2.direction));
    int result = 1;
    auto [topLeft, bottomRight] = SpatialUtils::getShipOccupancyRect(e2);
    for (int x = topLeft.x; x <= bottomRight.x; ++x)
    {
        for (int y = topLeft.y; y <= bottomRight.y; ++y)
        {
            if (isInSealane(Point2d(x, y)))
            {
                result += 1;
                break;
            }
        }
    }
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

// 初始化泊位到交货点的距离变量
std::vector<std::pair<int, int>> Map::initializeBerthToDeliveryDistances(BerthID berthId){
    std::vector<std::pair<int, int>> distsToDelivery;
    for(int deliveryId = 0;deliveryId < deliveryLocations.size(); deliveryId++){
        Point2d pos = deliveryLocations[deliveryId];
        distsToDelivery.push_back({deliveryId, maritimeBerthDistanceMap[berthId][pos.x][pos.y]});
    }
    // 升序排列
    std::sort(distsToDelivery.begin(), distsToDelivery.end(), [](std::pair<int, int>& a,std::pair<int, int> &b){
        return a.second < b.second;
    });
    return distsToDelivery;
}

MapFlag Map::getMapType()
{
    using namespace MapItemSpace;
    if (getCell(Point2d(197, 10)) == MapItem::DELIVERY_POINT &&
        getCell(Point2d(197, 191)) == MapItem::DELIVERY_POINT &&
        getCell(Point2d(101, 38)) == MapItem::ROBOT_SHOP &&
        getCell(Point2d(101, 157)) == MapItem::ROBOT_SHOP &&
        getCell(Point2d(181, 99)) == MapItem::ROBOT_SHOP &&
        getCell(Point2d(2, 2)) == MapItem::SHIP_SHOP &&
        getCell(Point2d(2, 197)) == MapItem::SHIP_SHOP)
    {
        LOGI("识别到图1");
        return MapFlag::MAP1;
    }
    else if (getCell(Point2d(2, 197)) == MapItem::DELIVERY_POINT &&
             getCell(Point2d(197, 197)) == MapItem::DELIVERY_POINT &&
             getCell(Point2d(46, 46)) == MapItem::ROBOT_SHOP &&
             getCell(Point2d(46, 148)) == MapItem::ROBOT_SHOP &&
             getCell(Point2d(146, 46)) == MapItem::ROBOT_SHOP &&
             getCell(Point2d(146, 148)) == MapItem::ROBOT_SHOP &&
             getCell(Point2d(3, 22)) == MapItem::SHIP_SHOP &&
             getCell(Point2d(196, 21)) == MapItem::SHIP_SHOP)
    {
        LOGI("识别到图2");
        return MapFlag::MAP2;
    }
    else
    {
        LOGI("识别到图3");
        return MapFlag::MAP3;
    }
    LOGW("未识别到地图类型");
    return MapFlag::NORMAL;
}

Direction Map::evaluateBestApproachDirection(const VectorPosition &shipPosition, const Point2d &pos)
{
    return Direction::EAST;
}


// 判断两艘船是否重合，在主航道上的体积不计入，true 是有重叠
bool Map::hasOverlap(VectorPosition &a, VectorPosition &b) {
    std::pair<Point2d, Point2d> ship1 = SpatialUtils::getShipOccupancyRect(a);
    std::pair<Point2d, Point2d> ship2 = SpatialUtils::getShipOccupancyRect(b);
    std::unordered_set<Point2d> shipSpace;
    // 遍历第一个船空间
    for (int x = ship1.first.x; x <= ship1.second.x; x++){
        for (int y = ship1.first.y; y <= ship1.second.y; y++){
            shipSpace.insert(Point2d(x, y));
        }
    }
    // 遍历第二个船空间，判断和第一个船空间是否重合
    for (int x = ship2.first.x; x <= ship2.second.x; x++){
        for (int y = ship2.first.y; y <= ship2.second.y; y++){
            // 重合并且不是主航道，返回true
            if(shipSpace.find(Point2d(x, y)) != shipSpace.end() && !isInSealane(Point2d(x, y)))
                return true;
        }
    }
    return false;
}

// 给定船核心点，判断船是否位于主航道
bool Map::isShipInSeaLane(VectorPosition &vecPos){
    std::pair<Point2d, Point2d> shipSpace = SpatialUtils::getShipOccupancyRect(vecPos);
    for (int x = shipSpace.first.x; x <= shipSpace.second.x; x++){
        for (int y = shipSpace.first.y; y <= shipSpace.second.y; y++){
            if (isInSealane({x, y})) 
                return true;
        }
    }
    return false;
}