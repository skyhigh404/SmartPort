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

bool Map::isBerthReachable(BerthID id, Point2d position)
{
    if (berthDistanceMap.at(id)[position.x][position.y] != INT_MAX)
        return true;
    return false;
}

std::vector<Point2d> Map::getChangedStates(int robotID) const
{
    std::vector<Point2d> obstacle;
    obstacle.reserve(5 * (ROBOTNUMS - 1));
    for (int i = 0; i < robotPosition.size(); ++i)
    {
        if (i == robotID)
            continue; // 不考虑自身
        for(int j = -1; j <= 1; ++j){
            for(int k = -1; k <= 1; ++k)
            {
                Point2d next = Point2d(robotPosition[i].get().x+j, robotPosition[i].get().y+k);
                if(inBounds(next) && passable(next))
                    obstacle.push_back(next);
            }
        }
        // obstacle.push_back(robotPosition[i]);
        // for (const Point2d &pos : neighbors(robotPosition[i]))
        //     obstacle.push_back(pos);
    }
    return obstacle;
}

std::string printVector(const std::vector<Point2d> &path)
{
    std::ostringstream oss;
    for (const auto &val : path)
        oss << val << " ";
    oss << "\n";
    return oss.str();
}