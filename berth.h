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
    int totalValue = 0; //泊位当前理论收益，每帧重新计算
public:
    Berth(int id, Point2d pos, int time, int velocity)
        : id(id), pos(pos), time(time), velocity(velocity), stockpile(0), stockpileValue(0)
    {
    }

    // 打印泊位信息
    void info(){
        std::string berth_info = "泊位" + std::to_string(id) + ",装货速度：" + std::to_string(velocity) + ",送货时间：" + std::to_string(time) + ";"+"剩余容量："+ std::to_string(residue_num)+";";
        std::string reach_info = "已到达货物数量:" + std::to_string(reached_goods.size()) + ";";
        for(const auto& good : reached_goods){
            reach_info += "(" + std::to_string(good.status) + "," + std::to_string(good.value) + "),";
        }

        std::string unreach_info = "未到达货物数量:" + std::to_string(unreached_goods.size()) + ";";
        for(const auto& good : unreached_goods){
            unreach_info += "(" + std::to_string(good.status) + "," + std::to_string(good.value) + "),";
        }
        LOGI(berth_info,reach_info,";",unreach_info);
    }

};
