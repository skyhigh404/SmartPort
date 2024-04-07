#pragma once
#include "map.h"

struct Params
{
    // 注意命名规范，防止不同的子类超参互相冲突
    // robot 参数
    // 例如，float greedyRobotScheduleTTLProfitWeight

    // 机器人调度超参
    float TTL_ProfitWeight = 1.5;
    int TTL_Bound = 500;
    bool PartitionScheduling = false; // 是否分区调度
    
    // 购买策略超参
    int maxRobotNum = 10;    // 最多购买机器人数目
    int maxShipNum = 10;     // 最多购买船只数目
    std::vector<std::vector<int>> robotPurchaseAssign = {{4, 4, 4, 5, 6, 7, 8, 9, 10}, {1, 2, 3, 4, 5, 6}, {1, 2, 3, 4, 5, 6}};
    std::vector<std::vector<int>> shipPurchaseAssign = {{4, 4, 4, 5, 6, 7, 8, 9, 10}, {1, 2, 3, 4, 5, 6}, {1, 2, 3, 4, 5, 6}};
    int startNum = 1;       // 最初的数目（机器人、轮船）

    //  泊位超参
    float ABLE_DEPART_SCALE = 0.15;           //可以去虚拟点的剩余容量比例
    int MAX_SHIP_NUM = 2;     // 一个泊位最多几艘船
    int TIME_TO_WAIT = 100; //等待有货的时间段
    int CAPACITY_GAP = 10;   // 泊位溢出货物量和船的容量差

    // ship 参数

    Params(MapFlag mapFalg) {
        if (mapFalg == MapFlag::NORMAL) {

        }
        else if (mapFalg == MapFlag::ERROR) {

        }
    }
};
