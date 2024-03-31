#pragma once

#include "scheduler.h"
#include <memory>

class FinalShipScheduler : public ShipScheduler
{
public:
    // 实现接口
    // todo 当前是局部最优，如果效果不佳，后续可以把多船分配泊位单独形成一类，通过模拟获取全局最优
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
        return SchedulerName::Final_SHIP_SCHEDULER;
    }

    FinalShipScheduler(const std::vector<int> &berthCluster, const std::vector<std::vector<Berth>> &clusters);

private:
    // 需要用到的超参数
    // 泊位超参数，需要搬到shipScheduler
    float ABLE_DEPART_SCALE;           //可以去虚拟点的剩余容量比例
    int MAX_SHIP_NUM;     // 一个泊位最多几艘船
    int TIME_TO_WAIT; //等待有货的时间段
    int CAPACITY_GAP;   // 泊位溢出货物量和船的容量差
    
    // 边界变量
    int maxCapacity = -1;
    int minVelocity = INT_MAX;
    int maxTime = -1;
    int maxLoadTime;
    int hasInit = false;

    // 聚簇的泊位
    std::shared_ptr<std::vector<std::vector<Berth>>> clusters;
    const std::shared_ptr<std::vector<int>> berthCluster;   // 每个泊位所对应的类

    std::unordered_map<int, int> backupToFinal; // 候选泊位对应的终局泊位
    std::unordered_map<int, int> finalToBackup; // 终局泊位对应的候选泊位
    std::unordered_map<int, int> shipToFinal; // 船对应的终局泊位
    std::unordered_map<int, int> finalToShip; // 终局泊位对应的船
    std::vector<Berth> finalBerths; //终局泊位
    std::vector<Berth> backupBerths;    //候选泊位

private:
    // 处理船在候选泊位的情况
    ShipActionSpace::ShipAction
    handleShipAtBackupBerth(Ship& ship, std::vector<Berth> &berths);
    // 处理船在终局泊位的情况
    ShipActionSpace::ShipAction
    handleShipAtFinalBerth(Ship& ship, std::vector<Berth> &berths);
    // 处理船在虚拟点的情况
    ShipActionSpace::ShipAction
    handleShipAtVirtualPoint(Ship& ship, std::vector<Berth> &berths);

    // 处理船不在指定泊位情况（只有开局会出现）
    ShipActionSpace::ShipAction
    handleShipNotAtAssignedBerth(Ship& ship, std::vector<Berth> &berths);
    // // 处理船前往候选泊位的情况
    // ShipActionSpace::ShipAction
    // handleShipEnRouteToBackupBerth(Ship& ship, std::vector<Berth> &berths);
    // // 处理船前往终局泊位的情况
    // ShipActionSpace::ShipAction
    // handleShipEnRouteToFinalBerth(Ship& ship, std::vector<Berth> &berths);

    // 初始化变量
    void init(std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods);

    // 配对终局泊位和候选泊位
    void selectFinalAndBackupBerths(std::vector<Berth> &berths);

    // 为所有船选定终局泊位
    void assignFinalBerthsToShips(std::vector<Ship> &ships, std::vector<Berth> &berths);

    // 为单个船分配终局泊位
    BerthID assignFinalBerth(Ship &ship);

    // 获取船的终局泊位
    BerthID getShipFinalBerth(Ship &ship);

    // 获取船的候选泊位
    BerthID getShipBackupBerth(Ship &ship);

    // 判断终局泊位是否已经分配
    bool hasAssigned(BerthID berthId);

    // 判断船是否在指定的候选泊位或者终局泊位上
    bool isShipAtAssignedBerth(Ship &ship);

    // 判断船是否在候选泊位上
    bool isShipAtBackupBerth(Ship &ship);

    // 判断船是否在前往候选泊位途中
    bool isShipOnRouteToBackBerth(Ship &ship);

    // 判断船是否在终局泊位上
    bool isShipAtFinalBerth(Ship &ship);

    // 判断泊位上是否有货物可装载
    bool isThereGoodsToLoad(Berth &berth);

    // 判断船是否有时间前往候选泊位
    bool canReachBackupBerth(Ship &ship, std::vector<Berth> &berths);

    // 判断船是否有时间前往终局泊位
    bool canReachFinalBerth(Ship &ship, std::vector<Berth> &berths);

    // 判断船是否必须前往终局泊位
    bool shouldReachFinalBerth(Ship &ship, std::vector<Berth> &berths);

    // 判断船是否能去虚拟点后再回来指定泊位，最后再前往虚拟点
    bool canDepartAndReturn(Ship &ship, Berth &berth, std::vector<Berth> &berths);

    bool FinalShipScheduler::shouldDepartAndReturn(Ship &ship, Berth &berth, std::vector<Berth> &berths);

    // 判断船是否必须前往虚拟点
    bool shouldDepartBerth(Ship &ship,std::vector<Berth> &berths);

    // // 禁用泊位
    // void disableBerth(Berth &berth);

    // 初始化泊位的状态
    void updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths,std::vector<Goods> & goods);

    // 装货
    void loadGoodAtBerth(Ship &ship, std::vector<Berth> &berths);

};
