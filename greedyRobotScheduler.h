#pragma once

#include "scheduler.h"

class GreedyRobotScheduler : public RobotScheduler
{
public:
    // 实现接口
    std::vector<std::pair<RobotID, RobotActionSpace::RobotAction>>
    scheduleRobots(const Map &map,
                   const std::vector<Robot> &robots,
                   std::vector<Goods> &goods,
                   const std::vector<Berth> &berths,
                   const int currentFrame) override;
    // 设置参数
    void setParameter(const Params &params) override;
    // 返回调度器名字
    SchedulerName getSchedulerName() override
    {
        return SchedulerName::Greedy_ROBOT_SCHEDULER;
    }

private:
    // 需要用到的超参数
    float TTLWeitht;
    // 等等
    std::vector<std::pair<BerthID, int>> maxRobotsPerBerth; // 记录每个泊位分配机器人的上限
private:
    // 辅助变量
    std::vector<std::pair<BerthID, int>> robotAllocationPerBerth; // 记录每个泊位已经分配了多少机器人

private:
    // 统计每个泊位分配了多少机器人，维护 robotAllocationPerBerth 变量
    void countRobotsPerBerth(const std::vector<Robot> &robots);
    // 判断机器人是否需要去拿货物
    bool shouldFetchGoods(const Robot &robot);
    // 判断机器人是否需要去泊位
    bool shouldMoveToBerth(const Robot &robot);

    // 对单个机器人寻找合适的货物
    std::pair<RobotID, RobotActionSpace::RobotAction>
    findGoodsForRobot(const Map &map,
                      const Robot &robot,
                      std::vector<Goods> &goods,
                      const std::vector<Berth> &berths,
                      const int currentFrame);

    // 对单个机器人寻找合适的泊位
    std::pair<RobotID, RobotActionSpace::RobotAction>
    findBerthForRobot(const Robot &robot,
                      const std::vector<Berth> &berths);

    // 根据 robotAllocationPerBerth 以及机器人对泊位的可达性和泊位是否启用，筛选出可用泊位
    std::vector<BerthID> getAvailableBerths(const Robot &robot);

    // 获取可用的货物子集
    std::vector<std::reference_wrapper<Goods>>
    getAvailableGoods(const std::vector<Goods> &goods,
                      std::vector<BerthID> &berthIDs);
};
