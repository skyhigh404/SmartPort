#pragma once

#include <string>
#include "utils.h"
#include "log.h"

class Ship
{
public:
    int id;
    int capacity;
    Point2d pos;
    int state;   // 0: 运输中, 1: 正常运行状态即装货状态或运输完成状态, 2: 泊位外等待状态
    int berthId; // 目标泊位 ID
    int now_capacity;   //船的剩余容量

    // 目前没有剩余容量标识
public:
    Ship(int id, int capacity) : id(id), capacity(capacity), berthId(-1), state(-1), now_capacity(capacity) {}

    std::string moveToBerth(int berthId)
    {
#ifdef DEBUG
        assert(berthId >= 0 && berthId <= 10);
#endif
        using namespace std::string_literals;
        return "ship "s + std::to_string(id) + " "s + std::to_string(berthId);
    }

    std::string go()
    {
        // 生成船移动到虚拟点的指令
        using namespace std::string_literals;
        return "go "s + std::to_string(id);
    }

    // 装货,并返回转货的数量
    int load(int num){
        LOGI("now_capacity before",this->now_capacity);
        if(now_capacity == 0){
            // 异常情况，满货船舶停滞在泊位
            LOGW("ID: ", id, " now_capacity: ", now_capacity, " berth_id: ", berthId);
            return 0;
        }
        else if(now_capacity >= num){
            now_capacity -= num;
            LOGI("搬运货物：",num);
            return num;
            LOGI("now_capacity after：",this->now_capacity);
        }else {
            now_capacity = 0;
            return now_capacity;
        }
    }

    // 打印信息
    void info(){
        LOGI("船只",id ,",状态",state,",装货量：",capacity, ",剩余容量：" ,now_capacity,",泊位id：",berthId, ";");
    }

};