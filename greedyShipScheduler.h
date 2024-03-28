#pragma once

#include "scheduler.h"

class GreedyShipScheduler : public ShipScheduler
{
public:
    // 实现接口
    std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>>
    scheduleShips(Map &map,
                  std::vector<Ship> &ships,
                  std::vector<Berth> &berths,
                  std::vector<Goods> &goods,
                  std::vector<Robot> &robots,
                  int currentFrame) override;
    // 设置参数
    void setParameter(const Params &params) override;
    // 返回调度器名字
    SchedulerName getSchedulerName() override
    {
        return SchedulerName::Greedy_ROBOT_SCHEDULER;
    }

private:
    // 需要用到的超参数
    // 泊位超参数，需要搬到shipScheduler
    float canGoScale = 0.15;           // < 可以去虚拟点的剩余容量比例
    float canMoveScale = 0.1;          // > 可以移动泊位的剩余容量比例
    const int MAX_SHIP_NUM = 2;     // 一个泊位最多几艘船
    int timeToWait = 10; //等待有货的时间段
    // 等等

private:
    // 初始化泊位的状态
    void updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths);

    // 计算一段时间内泊位的价值收益，同时考虑泊位自身前往虚拟点的时间
    float calculateBerthFutureValue(std::vector<Berth> &berths, std::vector<Goods> &goods,int timeSpan);

    // 判断船只是否需要前往虚拟点
    // 1. 容量满了； 2. 游戏快结束了
    bool shouldDepartBerth(const Ship &ship,std::vector<Berth> &berths);

    // 判断泊位上是否有货物可装载
    bool isThereGoodsToLoad(Berth &berth); 

    // 为船找到最佳泊位，返回泊位id
    BerthID findBestBerthForShip(const Ship &ship, const std::vector<Berth> &berths, const std::vector<Goods> &goods);

    // 判断船可以前往其他泊位
    bool canMoveBerth(const Ship &ship,const Berth &Berth);

    // 处理船在路途的情况
    // 是否可以中途前往其他泊位（收益更高）
    ShipActionSpace::ShipAction
    handleShipOnRoute(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    // 处理船在泊位上的情况
    ShipActionSpace::ShipAction
    handleShipAtBerth(Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    // 处理在虚拟点的情况
    ShipActionSpace::ShipAction
    handleShipInEnd(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    // 处理在泊位外等待的情况
    ShipActionSpace::ShipAction
    handleShipWaiting(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

};
