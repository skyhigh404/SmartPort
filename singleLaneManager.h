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
    void findSingleLanes( Map& map_temp) {
        Map map = Map(map_temp);
        std::vector<std::vector<int>> flagMap(map.rows,std::vector<int>(map.cols,0));   //标记是否访问过
        singleLanes.clear(); // 清除之前的结果
        auto ship_start = std::chrono::high_resolution_clock::now();
        LOGI("开始寻找单行路");
        for (int x = 0; x < map.rows; x++) {
            for (int y = 0; y < map.cols; y++) {
                // LOGI("当前坐标：",x,",",y);
                // LOGI("数量：",singleLanes.size());
                
                // if (flagMap[x][y] == 0 && map_temp.grid[x][y] == MapItemSpace::MapItem::SPACE) {
                if (flagMap[x][y] == 0 && (isSinglePos(map,Point2d(x,y),0)) ) {
                    // 检查向右的单行路
                    int nextX = x + 1;
                    while (nextX < map.rows && isSinglePos(map,Point2d(nextX,y),0) && flagMap[nextX][y] == 0) {
                        flagMap[nextX][y] = 1;
                        nextX++;
                    }
                    if (nextX - x > 1) { // 发现单行路
                        // singleLanes.push_back(SingleLane(Point2d(x, y), Point2d(std::min(nextX,map.rows -1), y)));
                        singleLanes.push_back(SingleLane(Point2d(std::max(x - 1,0), y), Point2d(std::min(nextX,map.rows -1), y)));
                    }
                }
                if (flagMap[x][y] == 0 && (isSinglePos(map,Point2d(x,y),0)) || (isSinglePos(map,Point2d(x,y),1))) {
                    // 检查向下的单行路
                    int nextY = y + 1;
                    while (nextY < map.cols &&map.grid[x][nextY] == MapItemSpace::MapItem::SPACE
                    && isSinglePos(map,Point2d(x,nextY),1) && flagMap[x][nextY] == 0) {
                        flagMap[x][nextY] = 1;
                        ++nextY;
                    }
                    if (nextY - y > 1) { // 发现单行路
                        singleLanes.push_back(SingleLane(Point2d(x, std::max(y - 1,0)), Point2d(x, std::min(nextY,map.cols - 1))));
                    }
                }
            }
        }
        auto ship_end = std::chrono::high_resolution_clock::now();
        LOGI("寻路时长:",std::chrono::duration_cast<std::chrono::milliseconds>(ship_end - ship_start).count(),"ms");
        LOGI("寻找完毕！！");
    }

    // 判断是否是单行格子
    // flag 0 横向;1 纵向
    bool isSinglePos(Map &map,Point2d pos,int flag){
        Point2d left,right;
        switch (flag)
        {
        case 0:
            left = Point2d(pos.x,pos.y - 1);
            right = Point2d(pos.x,pos.y + 1);
            break;

        case 1:
            left = Point2d(pos.x - 1,pos.y);
            right = Point2d(pos.x + 1,pos.y);
            break;

        default:
            break;
        }
            // 传入位置是空的，其他x y是障碍
            if(map.grid[pos.x][pos.y] == MapItemSpace::MapItem::SPACE
            && (!map.inBounds(left) || (map.grid[left.x][left.y] != MapItemSpace::MapItem::SPACE && map.grid[left.x][left.y] != MapItemSpace::MapItem::BERTH))
            && (!map.inBounds(right) || map.grid[right.x][right.y] != MapItemSpace::MapItem::SPACE && map.grid[left.x][left.y] != MapItemSpace::MapItem::BERTH)){
                // LOGI("left:",left,",right:",right);
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
