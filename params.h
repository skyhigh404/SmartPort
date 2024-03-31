#pragma once
#include "map.h"

struct Params
{
    // 注意命名规范，防止不同的子类超参互相冲突
    // robot 参数
    // 例如，float greedyRobotScheduleTTLProfitWeight
    float TTL_ProfitWeight;
    int TTL_Bound;
    bool PartitionScheduling; // 是否分区调度
    double a;
    double b;
    int c;
    int d;
    bool e;

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
