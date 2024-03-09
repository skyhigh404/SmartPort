#pragma once
#include "robot.h"
#include "map.h"
#include "utils.h"
#include <vector>

class Pathfinder {
public:
    // 寻路算法的输入是起始点和目标点的坐标，以及地图信息。输出是一系列坐标，表示路径。
    // 如果无法到达，返回一个空的路径。
    virtual std::vector<Point2d> findPath(const Point2d& start, const Point2d& goal, const Map& map) = 0;
};

class AStarPathfinder : public Pathfinder {
public:
    std::vector<Point2d> findPath(const Point2d& start, const Point2d& goal, const Map& map) override
    {}
};
