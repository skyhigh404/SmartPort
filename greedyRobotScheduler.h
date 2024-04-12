#pragma once
#include "scheduler.h"
#include <memory>

class GreedyRobotScheduler : public RobotScheduler
{
public:
    // 实现接口
    void scheduleRobots(const Map &map,
                        std::vector<Robot> &robots,
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
    // 初始化
    void initialize() {}

    GreedyRobotScheduler(std::vector<std::vector<Berth>> &_clusters, std::vector<int> &_berthCluster);

private:
    // 需要用到的超参数
    float TTL_ProfitWeight;
    int TTL_Bound;
    bool PartitionScheduling; // 是否分区调度
    int maxRobotNum;
    std::vector<int> ASSIGNBOUND;
    float robotReleaseBound;
    bool DynamicPartitionScheduling;
    int DynamicSchedulingInterval;
    int startPartitionScheduling;
    // 等等
    std::vector<std::pair<BerthID, int>> maxRobotsPerBerth; // 记录每个泊位分配机器人的上限
private:
    // 辅助变量
    std::vector<std::pair<BerthID, int>> robotAllocationPerBerth; // 记录每个泊位已经分配了多少机器人
    std::vector<std::vector<Berth>> clusters;                     // 每个簇对应的泊位
    std::shared_ptr<std::vector<int>> berthCluster;         // 每个泊位所对应的类
    std::vector<int> assignment;
    int lastReassignFrame = 0; //上次动态调度的时刻
    bool enterFinal; // 判断是否进入终局
    bool allAssign = false;

private:
    // 根据类来分配机器人
    void assignRobotsByCluster(vector<Robot> &robots, const Map &map, vector<int> assignBound = vector<int>());
    // 根据类来重新分配机器人
    void reassignRobotsByCluster(vector<Goods> &goods, vector<Robot> &robots, const Map &map, const std::vector<Berth> &berths);
    // 统计每个泊位分配了多少机器人，维护 robotAllocationPerBerth 变量
    void countRobotsPerBerth(const std::vector<Robot> &robots);
    // 判断机器人是否需要去拿货物
    bool shouldFetchGoods(const Robot &robot);
    // 判断机器人是否需要去泊位
    bool shouldMoveToBerth(const Robot &robot);

    // 对单个机器人寻找合适的货物
    void
    findGoodsForRobot(const Map &map,
                      Robot &robot,
                      std::vector<Goods> &goods,
                      const std::vector<Berth> &berths,
                      const int currentFrame);

    // 对单个机器人寻找合适的泊位
    void
    findBerthForRobot(Robot &robot,
                      std::vector<Goods> &goods,
                      const std::vector<Berth> &berths,
                      const Map &map);

    // 根据 robotAllocationPerBerth 以及机器人对泊位的可达性和泊位是否启用，筛选出可用泊位
    std::vector<BerthID> getAvailableBerths(const Robot &robot);

    // 获取可用的货物子集
    std::vector<std::reference_wrapper<Goods>>
    getAvailableGoods(std::vector<Goods> &goods);

    // 确定机器人在泊位还是不在泊位
    int WhereIsRobot(const Robot &robot, const std::vector<Berth> &berths, const Map &map);
    // 计算机器人到货物的距离
    vector<long long> Cost_RobotToGood(const Robot &robot,
                                       std::vector<std::reference_wrapper<Goods>> &availableGoods,
                                       const std::vector<Berth> &berths,
                                       const Map &map);
    // 计算货物到最佳泊位的距离
    vector<long long> Cost_GoodToBerth(std::vector<std::reference_wrapper<Goods>> &availableGoods,
                                       const Map &map);
    // 计算收益并排序
    std::pair<vector<float>, vector<int>> 
    getProfitsAndSortedIndex(std::vector<std::reference_wrapper<Goods>>& availableGoods,
                             vector<long long>& cost_robot2good,
                             vector<long long>& cost_good2berth);
};
