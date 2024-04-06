#pragma once
#include <vector>
#include <numeric>
#include "goods.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "berth.h"
#include "params.h"
#include "utils.h"

enum class SchedulerName
{
    Greedy_ROBOT_SCHEDULER,
    Greedy_SHIP_SCHEDULER,
    Final_SHIP_SCHEDULER
};

using std::vector;

class RobotScheduler
{
public:
    // 总的调度函数，在子类里进一步封装实现
    virtual void
    scheduleRobots(const Map &map,
                   std::vector<Robot> &robots,
                   std::vector<Goods> &goods,
                   const std::vector<Berth> &berths,
                   const int currentFrame) = 0;
    // 设置参数，参数定义在子类里
    virtual void setParameter(const Params &params) = 0;
    // 返回调度器名字
    virtual SchedulerName getSchedulerName() = 0;
    // 初始化
    virtual void initialize() = 0;
    virtual ~RobotScheduler() {}
};

class ShipScheduler
{
public:
    // 总的调度函数，在子类里进一步封装实现
    virtual void
    scheduleShips(Map &map,
                  std::vector<Ship> &ships,
                  std::vector<Berth> &berths,
                  std::vector<Goods> &goods,
                  std::vector<Robot> &robots) = 0;
    // 设置参数，参数定义在子类里
    virtual void setParameter(const Params &params) = 0;
    // 返回调度器名字
    virtual SchedulerName getSchedulerName() = 0;
    // 初始化
    virtual void initialize() = 0;
    virtual ~ShipScheduler() {}
};