#pragma once
#include "map.h"

struct Params
{
    // 注意命名规范，防止不同的子类超参互相冲突
    // robot 参数
    float TTL_ProfitWeight;
    int TTL_Bound;
    bool PartitionScheduling; // 是否分区调度
    double a;
    double b;
    int c;
    int d;
    bool e;

    // ship 参数

    Params(MapFlag mapFalg) {
        if (mapFalg == MapFlag::NORMAL) {

        }
        else if (mapFalg == MapFlag::LABYRINTH) {

        }
        else if (mapFalg == MapFlag::ERROR) {

        }
    }
};
