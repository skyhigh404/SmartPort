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

public:
    // 单个泊位维护的状态变量
    bool isEnabled; // 标识泊位是否启用
    int category;   // 标识该泊位聚类后的类别
    int stockpile;                      // 泊位堆积的货物量
    int stockpileValue;                 // 泊位堆积的货物的价值
    std::vector<Goods> reached_goods;   // 堆积货物的列表
    std::vector<Goods> unreached_goods; // 未到达货物的列表
    int residue_num = 0;                // 泊位当前剩余无法装在的货物数量，每帧重新计算
    int totalValue = 0;                 // 泊位当前理论收益，每帧重新计算
    int shipInBerthNum = 0;             // 泊位上船的数量

    // todo ,所有泊位的状态变量，判断是否删除
    static std::vector<bool> available_berths;

    std::vector<std::vector<int>> storageSlots; // 16个格子，-1表示没有机器人，否则表示机器人id
public:
    // 统计变量
    static int totalLoadGoodnum; // 总装货的数量
    static int maxLoadGoodNum;   // 理论最大装货数量
    static int deliverGoodNum;   // 送达货物数量

    // // todo，泊位超参数，需要搬到shipScheduler
    // float canGoScale = 0.15;           // < 可以去虚拟点的剩余容量比例
    // float canMoveScale = 0.1;          // > 可以移动泊位的剩余容量比例
    // const static int MAX_SHIP_NUM = 2; // 一个泊位最多几艘船

    Berth(int id, Point2d pos, int time, int velocity)
        : id(id), 
        pos(pos), 
        time(time), 
        velocity(velocity), 
        isEnabled(false),
        category(-1),
        stockpile(0), 
        stockpileValue(0)
    {
        storageSlots = std::vector<std::vector<int>>(4, std::vector<int>(4, -1));
    }

    // 打印泊位信息
    void info()
    {
        std::string berth_info = "泊位" + std::to_string(id) + "位置" + std::to_string(pos.x) + ',' + std::to_string(pos.y) + ",装货速度：" + std::to_string(velocity) + ",送货时间：" + std::to_string(time) + ";" + "溢出容量：" + std::to_string(residue_num) + ";";
        std::string reach_info = "已到达货物数量:" + std::to_string(reached_goods.size()) + ";";
        // for(const auto& good : reached_goods){
        //     reach_info += "(" + std::to_string(good.id) + "," + std::to_string(good.value) + "),";
        // }

        std::string unreach_info = "未到达货物数量:" + std::to_string(unreached_goods.size()) + ";总价值：" + std::to_string(totalValue);
        // for(const auto& good : unreached_goods){
        //     unreach_info += "(" + std::to_string(good.id) + "," + std::to_string(good.value) + "),";
        // }
        LOGI(berth_info, reach_info, ";", unreach_info);
        LOGI("总装货量：", totalLoadGoodnum, ",送达货物量：", deliverGoodNum, ",理论最大装货量：", maxLoadGoodNum, ", 成功装载比例：", totalLoadGoodnum * 1.0 / maxLoadGoodNum, ",成功送达比例：", deliverGoodNum * 1.0 / maxLoadGoodNum);
        // std::vector<std::vector<int>> temp(4,std::vector<int>(4,-1));
        // for(int i =0;i < 4;i++){
        //     for(int j=0;j<4;j++){
        //         if(storageSlots[i][j] != nullptr){
        //             temp[i][j] = storageSlots[i][j]->id;
        //         }
        //     }
        // }
        // for(int i=0;i<4;i++){
        //     LOGI(storageSlots[i][0]," ",storageSlots[i][1]," ",storageSlots[i][2]," ",storageSlots[i][3]);
        // }
        LOGI("-----------------------------------------------------------------------------------------------------------------------------------------------");
    }

    // 传入卸货数量，按照进货数量进行卸货
    void unloadGoods(int res)
    {
        int num = 0;
        for (int index = 0; index < res && index < reached_goods.size(); index++)
        {
            bool find_flag = false;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    if (storageSlots[i][j] == reached_goods[index].id)
                    {
                        storageSlots[i][j] = -1;
                        find_flag = true;
                        num++;
                        break;
                    }
                }
                if (find_flag)
                    break;
            }
        }
        totalLoadGoodnum += res;
        if (res != num)
        {
            LOGI("应该卸货数量：", res, ",实际卸货数量：", num);
            info();
        }
    }

    // 剩余时间能够再去一次泊位后再去虚拟点，预留时间10帧
    // todo 调参，缓冲时间应该等于 船容量/目的泊位的搬运速度
    bool canMoveBerth(int remainder = 0)
    {
        if (remainder - 500 - time >= 10)
        {
            return true;
        }
        return false;
    }

    // 是否必须要去虚拟点, +5帧缓冲时间
    // otherTime表示需要预留的时间
    bool mustGo(int remainder, int otherTime = 0)
    {
        if (remainder - time - otherTime <= 2 && remainder - time - otherTime >= -1)
        {
            return true;
        }
        return false;
    }
};
