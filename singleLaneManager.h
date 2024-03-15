#pragma once

#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include "utils.h"
#include "map.h"
#include "robot.h"

//  单行路
struct SingleLane {
public:
    Point2d start; // 单行路起点坐标
    Point2d end;   // 单行路终点坐标
    bool hasRobot; // 标志位，表示当前是否有机器人在单行路上

    SingleLane(Point2d start, Point2d end, bool hasRobot = false) : start(start), end(end), hasRobot(hasRobot) {}
};

class SingleLaneManager {
private:
    std::vector<SingleLane> singleLanes; // 存储所有单行路的集合

public:
    SingleLaneManager(){
        
    }

    SingleLaneManager(Map &map){
        findSingleLanes(map);
    }

    // 初始化地图中的单行路
    void findSingleLanes(Map map_temp) {
        Map map = Map(map_temp);
        singleLanes.clear(); // 清除之前的结果
        
        for (int x = 0; x < map.rows; ++x) {
            for (int y = 0; y < map.cols; ++y) {
                if (map.grid[x][y] == MapItemSpace::MapItem::SPACE) {
                    // 检查向下的单行路
                    int nextY = y + 1;
                    while (nextY < map.cols && map.grid[x][nextY] == MapItemSpace::MapItem::SPACE
                    && isSinglePos(map,Point2d(x,nextY),3)) {
                        map.grid[x][nextY] = MapItemSpace::MapItem::OBSTACLE;
                        ++nextY;
                    }
                    if (nextY - y > 1) { // 发现单行路
                        singleLanes.emplace_back(Point2d(x, y), Point2d(x, nextY - 1));
                    }

                    // 检查向右的单行路
                    int nextX = x + 1;
                    while (nextX < map.rows && map.grid[nextX][y] == MapItemSpace::MapItem::SPACE
                    && isSinglePos(map,Point2d(nextX,y),0)) {
                        map.grid[nextX][y] = MapItemSpace::MapItem::OBSTACLE;
                        ++nextX;
                    }
                    if (nextX - x > 1) { // 发现单行路
                        singleLanes.emplace_back(Point2d(x, y), Point2d(nextX - 1, y));
                    }
                    // 检查本格子 todo

                }
            }
        }
    }

    // 判断是否是单行格子
    // flag 3 向下；2 向上；0 向右；1 向左
    bool isSinglePos(Map &map,Point2d pos,int flag){
        Point2d x,y;
        switch (flag)
        {
        case 3:
            x = Point2d(pos.x - 1,pos.y);
            y = Point2d(pos.x + 1,pos.y);
            break;
        case 2:
            x = Point2d(pos.x - 1,pos.y);
            y = Point2d(pos.x + 1,pos.y);
            break;
        case 0:
            x = Point2d(pos.x,pos.y - 1);
            y = Point2d(pos.x,pos.y + 1);
            break;

        case 1:
            x = Point2d(pos.x,pos.y - 1);
            y = Point2d(pos.x,pos.y + 1);
            break;

        default:
            break;
        }
            // 传入位置是空的，其他x y是障碍
            if(map.grid[pos.x][pos.y] == MapItemSpace::MapItem::SPACE
            && (map.grid[x.x][x.y] != MapItemSpace::MapItem::SPACE || map.inBounds(x))
            && (map.grid[y.x][y.y] != MapItemSpace::MapItem::SPACE || !map.inBounds(y))){
                return true;
        }
        return false;
    }

    const std::vector<SingleLane>& getSingleLanes() const {
        return singleLanes;
    }

    std::vector<Point2d> getSingleLanesVector() const {
        std::vector<Point2d> res;
        for(auto &singleLane : singleLanes){
            res.push_back(singleLane.start);
            res.push_back(singleLane.end);
        }
        return res;
    }

    // 判断机器人是否即将进入单行路，并根据hasRobot判断是否通行
    bool canRobotEnter(const Robot& robot) {
        Point2d robotPos = robot.pos;
        for (const auto& lane : singleLanes) {
            // 检查机器人当前位置是否在单行路的一个端点附近
            if (robotPos.x == lane.start.x && robotPos.y == lane.start.y - 1 ||
                robotPos.x == lane.start.x && robotPos.y == lane.start.y + 1 ||
                robotPos.x == lane.end.x && robotPos.y == lane.end.y - 1 ||
                robotPos.x == lane.end.x && robotPos.y == lane.end.y + 1) {
                // 如果机器人即将进入单行路，检查该路是否已有机器人占用
                return !lane.hasRobot;
            }
        }
        return true; // 如果不是单行路，或单行路没有被占用，返回true
    }

    // 更新地图中单行路的hasRobot
    void updateSingleLanesWithRobots(const std::vector<Robot>& robots) {
        // 首先，假设所有单行路上没有机器人
        for (auto& lane : singleLanes) {
            lane.hasRobot = false;
        }

        // 检查每个机器人，如果它在某条单行路上，更新该单行路的hasRobot状态
        for (const auto& robot : robots) {
            for (auto& lane : singleLanes) {
                // 检查机器人是否在该单行路上
                if (isRobotOnLane(robot.pos, lane)) {
                    lane.hasRobot = true;
                    break; // 一个机器人只能在一条单行路上，所以找到后就可以停止检查
                }
            }
        }
    }

private:
    // 辅助函数：检查机器人是否在给定的单行路上
    bool isRobotOnLane(const Point2d& robotPos, const SingleLane& lane) {
        // 这里的实现需要根据单行路的具体定义，以下是一种可能的简化实现
        // 假设单行路是直线，只检查机器人是否在单行路的起点和终点之间
        if (lane.start.x == lane.end.x) { // 垂直单行路
            return robotPos.x == lane.start.x && robotPos.y >= lane.start.y && robotPos.y <= lane.end.y;
        } else if (lane.start.y == lane.end.y) { // 水平单行路
            return robotPos.y == lane.start.y && robotPos.x >= lane.start.x && robotPos.x <= lane.end.x;
        }
        return false;
    }
};
