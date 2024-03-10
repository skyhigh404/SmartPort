#include "scheduler.h"
#include "pathFinder.h"

using std::vector;

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    // 寻路
    vector<vector<vector<Point2d>>> path2goods(robots.size(), vector<vector<Point2d>>(goods.size(), vector<Point2d>())), \
                                    path2berths(robots.size(), vector<vector<Point2d>>(berths.size(), vector<Point2d>())); //机器人到达货物\泊位的路径
    for (int i=0;i<robots.size();i++) {
        for (int j=0;j<goods.size();j++) {
            path2goods[i][j] = AStarPathfinder::findPath(robots[i].pos, goods[j].pos, map);
        }
        for (int j=0;j<berths.size();j++) {
            path2berths[i][j] = AStarPathfinder::findPath(robots[i].pos, berths[j].pos, map);
        }
    }
    vector<vector<vector<Point2d>>> path2berth(goods.size(), vector<vector<Point2d>>(berths.size(), vector<Point2d>())); //货物到泊位的路径
    for (int i=0;i<goods.size();i++) {
        for (int j=0;j<berths.size();j++) {
            path2berth[i][j] = AStarPathfinder::findPath(goods[i].pos, berths[j].pos, map);
        }
    }
    
    // 衡量收益：机器人（未携带货物）<->货物<->泊位 | 机器人（携带货物）<->泊位
    // 收益 = 货物利润 / 机器人将货物送达泊位的耗时
    vector<int> robots_gain(robots.size(), int());
    for (int i=0;i<robots.size();i++) {
        if (robots[i].carryingItem) {
            // robots_gain[i] = 
        }
    }
    
    // 确定\分配机器人目的地
    // 调度机器人（寻路+碰撞检测+避让+指令生成）
    // 避让：多个人导致的（用于找出相关机器人），undo，避让代价+不避让损失：
    return std::vector<std::pair<int, Action>>();
}

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths)
{
    return std::vector<std::pair<int, Action>>();
}