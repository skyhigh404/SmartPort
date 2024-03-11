#include "scheduler.h"
#include "pathFinder.h"

using std::vector;

int dist(Point2d a, Point2d b) {return abs(a.x-b.x)+abs(a.y-b.y);}

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    // 寻路
    AStarPathfinder pathfinder;
    vector<vector<std::variant<Path, PathfindingFailureReason>>> path2goods(robots.size(), vector<std::variant<Path, PathfindingFailureReason>>(goods.size(), std::variant<Path, PathfindingFailureReason>())), \
                                    path2berths(robots.size(), vector<std::variant<Path, PathfindingFailureReason>>(berths.size(), std::variant<Path, PathfindingFailureReason>())); //机器人到达货物\泊位的路径
    vector<vector<int>> cost2goods(robots.size(), vector<int>(goods.size())), \
                        cost2berths(robots.size(), vector<int>(berths.size()));
    for (int i=0;i<robots.size();i++) {
        for (int j=0;j<goods.size();j++) {
            path2goods[i][j] = pathfinder.findPath(robots[i].pos, goods[j].pos, map);
            if (std::holds_alternative<Path>(path2goods[i][j]))
                cost2goods[i][j] = std::get<Path>(path2goods[i][j]).size();
        }
    }
    // vector<vector<std::variant<Path, PathfindingFailureReason>>> path2berths(goods.size(), vector<std::variant<Path, PathfindingFailureReason>>(berths.size(), std::variant<Path, PathfindingFailureReason>())); //货物到泊位的路径
    for (int i=0;i<goods.size();i++) {
        for (int j=0;j<berths.size();j++) {
            path2berths[i][j] = pathfinder.findPath(goods[i].pos, berths[j].pos, map);
            if (std::holds_alternative<Path>(path2goods[i][j]))
                cost2berths[i][j] = std::get<Path>(path2berths[i][j]).size();
        }
    }

    // 衡量收益
    // 计算每个机器人将货物送达泊位的耗时
    std::vector<std::vector<int>> profits(robots.size(), std::vector<int>(goods.size(), 0));
    vector<int> bestBerthIndex(berths.size(), -1);
    for (int i = 0; i < robots.size(); i++) {
        for (int j = 0; j < goods.size(); j++) {
            if (robots[i].carryingItem && robots[i].carryingItemId != goods[j].id) // j or goods[j].id?
                continue;
            // 计算机器人从当前位置到达货物处取货的耗时
            int timeToGoods = cost2goods[i][j];
            int time = INT_MAX;
            // 计算机器人从货物处取货到达泊位的耗时
            for (int k = 0; k < berths.size(); k++) {
                // 如果机器人无法到达该泊位，则跳过该泊位
                if (cost2berths[i][k] == INT_MAX) {
                    continue;
                }
                if (cost2berths[i][k] < time) {
                    time = cost2berths[i][k];
                    bestBerthIndex[j] = k;
                }
                // 计算总耗时
                int totalTime = timeToGoods + cost2berths[i][k];
                // 计算货物的利润
                int profit = goods[j].value;
                // 计算收益
                profits[i][j] = profit / totalTime;
            }
        }
    }

    // 确定\分配机器人目的地
    std::vector<std::pair<int, int>> robotDestinations(robots.size(), std::make_pair(-1, -1)); // (货物索引, 泊位索引)
    std::vector<vector<int>> indices(robots.size(), std::vector<int>(goods.size(), 0));
    std::vector<std::pair<int, Action>> robotActions; // (货物索引, 泊位索引)
    for (int i = 0; i < robots.size(); ++i) {
        for (int j=0;j<goods.size();j++) {
            indices[i][j] = j;
        }
        std::sort(indices[i].begin(), indices[i].end(), [&](int a, int b) {
            return profits[i][a] > profits[i][b]; // 根据第二个维度进行降序排序
        });
        std::sort(profits[i].begin(), profits[i].end(), [&](int a, int b) {
            return profits[i][a] > profits[i][b]; // 根据第二个维度进行降序排序
        });

        for (int j = 0; j < goods.size(); ++j) {
            int goodsIndex = indices[i][j];
            int berthsIndex = bestBerthIndex[goodsIndex];

            if (robots[i].carryingItem && robots[i].carryingItemId!=j) continue; // 机器人和货物不匹配
            if (!robots[i].carryingItem && goods[goodsIndex].status!=0) continue; //货物不可分配

            // 携带货物但离泊位还远
            if (robots[i].status==MOVING_TO_BERTH && dist(robots[i].pos, berths[berthsIndex].pos)>6) {
                // robotDestinations[i] = std::make_pair(goodsIndex, berthsIndex);
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, berths[berthsIndex].pos, berths[berthsIndex].id}));
                break;
            } 
            // 携带货物且离泊位近，开始分配具体放货位置
            else if (robots[i].status==MOVING_TO_BERTH && dist(robots[i].pos, berths[berthsIndex].pos)<=6) {
                Point2d berths_pos=berths[berthsIndex].pos;
                int stockpile=berths[berthsIndex].stockpile;
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, Point2d(berths_pos.x+stockpile/4, berths_pos.y+stockpile%4), berths[berthsIndex].id}));
                robots[i].status = UNLOADING;
                break;
            }
            // 即将到达卸货地点，到达并卸货
            else if (robots[i].status==UNLOADING && dist(robots[i].pos, robots[i].path[-1])==1) {
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, robots[i].path[-1], berths[berthsIndex].id}));
                robotActions.push_back(std::make_pair(i, Action{DROP_OFF_GOODS, robots[i].path[-1], berths[berthsIndex].id}));
                robots[i].status = IDLE;
                break;
            }
            // 如果该货物未被分配，并且该机器人未被分配，则将该机器人分配给该货物和泊位
            else if  (robots[i].status==IDLE && goods[goodsIndex].status==0) {
                // robotDestinations[i] = std::make_pair(goodsIndex, berthsIndex);
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id}));
                pickup[i] = goodsIndex;
                robots[i].status = MOVING_TO_GOODS;
                break;
            }
            // 机器人即将抵达取货地点，到达并取货
            else if (robots[i].status==MOVING_TO_GOODS && dist(robots[i].pos, robots[i].path[-1])==1) {
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, robots[i].path[-1], pickup[i]})); //goodsid
                robotActions.push_back(std::make_pair(i, Action{PICK_UP_GOODS, robots[i].path[-1], pickup[i]}));
                robots[i].status = MOVING_TO_BERTH;
                break;
            }
        }
    }

    // 调度机器人（寻路+碰撞检测+避让+指令生成）
    /*
    for (int i = 0; i < robots.size(); i++) {
        // 获取机器人要前往的目的地
        int goodsIndex = robotDestinations[i].first;
        int berthIndex = robotDestinations[i].second;
        
        // 根据机器人的状态生成不同的动作
        if (robots[i].status == RobotStatus::IDLE) {
            // 如果机器人当前处于空闲状态，那么机器人就前往取货
            robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id}));
            // 更新机器人状态
            robots[i].status = RobotStatus::MOVING_TO_GOODS;
        }
        else if (robots[i].status == RobotStatus::MOVING_TO_GOODS) {
            // 如果机器人正在前往取货的途中，检查是否已经到达目的地
            if (robots[i].pos == goods[goodsIndex].pos) {
                // 如果到达目的地，那么机器人就开始取货
                robotActions.push_back(std::make_pair(i, Action{PICK_UP_GOODS, goods[goodsIndex].pos, goods[goodsIndex].id}));
                // 更新机器人状态
                robots[i].status = RobotStatus::PICKING_UP;
            } else {
                // 如果还没有到达目的地，那么继续前往
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id}));
            }
        }
        else if (robots[i].status == RobotStatus::PICKING_UP) {
            // 如果机器人正在取货，检查是否已经完成取货
            if (robots[i].carryingItem) {
                // 如果已经取货成功，那么机器人就前往泊位送货
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, berths[berthIndex].pos, berths[berthIndex].id}));
                // 更新机器人状态
                robots[i].status = RobotStatus::MOVING_TO_BERTH;
            } else {
                // 如果取货失败，这里可以添加一些错误处理逻辑
            }
        }
        else if (robots[i].status == RobotStatus::MOVING_TO_BERTH) {
            // 如果机器人正在前往泊位的途中，检查是否已经到达目的地
            if (robots[i].pos == berths[berthIndex].pos) {
                // 如果到达泊位，那么机器人就完成任务
                robotActions.push_back(std::make_pair(i, Action{DROP_OFF_GOODS, berths[berthIndex].pos, berths[berthIndex].id}));
                // 更新机器人状态
                robots[i].status = RobotStatus::UNLOADING;
            } else {
                // 如果还没有到达目的地，那么继续前往
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, berths[berthIndex].pos, berths[berthIndex].id}));
            }
        } else if (robots[i].status == RobotStatus::UNLOADING) {
            // 如果机器人正在卸货，等待卸货完成
            // 这里可以添加一些逻辑，例如等待一定的时间后更新机器人状态
        }
    }
    // 避让：多个人导致的（用于找出相关机器人），undo，避让代价+不避让损失：
    */
    return robotActions;
}

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths)
{
    return std::vector<std::pair<int, Action>>();
}