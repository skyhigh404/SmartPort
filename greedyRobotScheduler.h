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

private:
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
};
