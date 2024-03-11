#pragma once

#include "utils.h"

class Berth
{
    // 泊位间移动时间 500 帧
public:
    int id;
    Point2d pos;  // 泊位左上角的坐标，泊位是一个 4x4 的矩形
    int time;     // 该泊位轮船运输到虚拟点的时间(虚拟点移动到泊位的时间同)，即产生价值的时间，时间用帧数表示。
    int velocity; // Velocity(1 <= Velocity <= 5)表示该泊位的装载速度，即每帧可以装载的物品数。

    int stockpile;      // 泊位堆积的货物量
    int stockpileValue; // 泊位堆积的货物的价值
    std::vector<Goods> reached_goods;  //堆积货物的列表
    std::vector<Goods> unreached_goods;  //未到达货物的列表
    int residue_num = 0;    //泊位当前剩余无法装在的货物数量，每帧重新计算
public:
    Berth(int id, Point2d pos, int time, int velocity)
        : id(id), pos(pos), time(time), velocity(velocity), stockpile(0), stockpileValue(0)
    {
    }
    
    int calculate_revenue(){

    }
};
