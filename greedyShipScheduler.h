#pragma once

#include "scheduler.h"

class GreedyShipScheduler : public ShipScheduler
{
public:
    // 实现接口
    // todo 当前是局部最优，如果效果不佳，后续可以把多船分配泊位单独形成一类，通过模拟获取全局最优
    void scheduleShips(Map &map,
                  std::vector<Ship> &ships,
                  std::vector<Berth> &berths,
                  std::vector<Goods> &goods,
                  std::vector<Robot> &robots) override;
    // 设置参数
    void setParameter(const Params &params) override;
    // 返回调度器名字
    SchedulerName getSchedulerName() override
    {
        return SchedulerName::Greedy_SHIP_SCHEDULER;
    }

    // 初始化
    void initialize()override {}

private:
    // 需要用到的超参数
    // 泊位超参数，需要搬到shipScheduler
    float ABLE_DEPART_SCALE;           //可以去虚拟点的剩余容量比例
    int MAX_SHIP_NUM;     // 一个泊位最多几艘船
    int TIME_TO_WAIT; //等待有货的时间段
    int CAPACITY_GAP;   // 泊位溢出货物量和船的容量差
    // 等等

private:
    // 初始化泊位的状态
    void updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths,std::vector<Goods> & goods);

    // 根据货物距离泊位距离计算货物价值
    float calculateGoodValueByDist(Goods &good);

    // // 计算一段时间内泊位的价值收益，同时考虑泊位自身前往虚拟点的时间
    // float calculateBerthFutureValue(std::vector<Berth> &berths, std::vector<Goods> &goods,int timeSpan);

    // 判断船只是否需要前往虚拟点
    // 1. 容量满了； 2. 游戏快结束了
    bool shouldDepartBerth( Ship &ship,std::vector<Berth> &berths);

    // 判断泊位上是否有货物可装载
    bool isThereGoodsToLoad(Berth &berth); 

    // 判断泊位最近有没有货物到来
    bool isGoodsArrivingSoon(Berth &berth, std::vector<Goods> goods); 

    // 为船找到最佳泊位，返回泊位id
    BerthID findBestBerthForShip(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods);

    // 为船找到最佳泊位，返回泊位id
    BerthID findBestBerthForShip(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods);

    // 当船在虚拟点时，选择最佳调度策略
    void scheduleShipAtDelivery(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods);

    // 当船在泊位时（没货），选择最佳调度策略（去泊位|去虚拟点）
    void scheduleFreeShipAtBerth(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods);

    // 当船在购买点时
    void scheduleShipAtShipShops(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods);

    // 计算船在该泊位上能得到多少收益（只考虑泊位上已有的货物）
    std::pair<int, int> calculateShipProfitInBerth(Map &map, Ship &ship, Berth &berth);

    // 判断船可以前往其他泊位
    bool canMoveBerth(Map &map, Ship &ship,Berth &Berth);

    void handleShipOnRoute(Map& map, Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    void handleShipAtBerth(Map &map, Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    // 处理在虚拟点的情况
    ShipActionSpace::ShipAction
    handleShipInEnd( Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    // 处理在泊位外等待的情况
    ShipActionSpace::ShipAction
    handleShipWaiting( Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods);

    // 比较船去两个泊位的收益
    bool compareBerthsValue(Berth &a,Berth &b);

    // 当船从当前泊位移动到其他泊位时，更新泊位相关参数
    void updateBerthWhereShipMove(Ship &ship,std::vector<Berth> &berths,BerthID targetId);

    // 分配最近的交货点
    int allocateDelivery(Berth &berth);

};
