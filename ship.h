#pragma once

#include <string>
#include "utils.h"
#include "log.h"

class Ship
{
public:
    int id;
    int capacity; // 船的容量
    int state;    // 0: 运输中, 1: 正常运行状态即装货状态或运输完成状态, 2: 泊位外等待状态
    int berthId;  // 目标泊位 ID
public:
    int now_capacity;           // 船的剩余容量
    int remainingTransportTime; // 船到目标泊位的剩余运行时间，在处理每一帧信息时维护

    // bool ableGo = true;   // 在运输期间，是否可以去虚拟点
    // bool ableMove = true; // 在运输期间，是否可以移动去其他泊位

public:
    Ship(int id, int capacity)
        : id(id),
          capacity(capacity),
          state(-1),
          berthId(-1),
          now_capacity(capacity),
          remainingTransportTime(0) {}

    // 生成移动到指定泊位的指令
    std::string moveToBerth(int berthId)
    {
        #ifdef DEBUG
        assert(berthId >= 0 && berthId < 10);
        #endif
        using namespace std::string_literals;
        // todo,重置船的剩余运行时间(500到时候置为全局参数)
        remainingTransportTime = 500;
        return "ship "s + std::to_string(id) + " "s + std::to_string(berthId);
    }

    // 生成船移动到虚拟点的指令
    // time为当前泊位前往虚拟点的时间
    std::string go(int time)
    {
        using namespace std::string_literals;
        // 重置船的剩余运行时间
        remainingTransportTime = time;
        reset();
        return "go "s + std::to_string(id);
    }

    void reset()
    {
        // 恢复状态
        now_capacity = capacity;
        state = 0;
        berthId = -1;
    }

    // 装货,并返回转货的数量
    int loadGoods(int num)
    {
        #ifdef DEBUG
        assert(now_capacity >= 0);
        #endif
        // LOGI("now_capacity before",this->now_capacity);
        if (now_capacity == 0)
        {
            // 异常情况，满货船舶停滞在泊位
            // LOGW("ID: ", id, " now_capacity: ", now_capacity, " berth_id: ", berthId);
            return 0;
        }
        else if (now_capacity >= num)
        {
            now_capacity -= num;
            // LOGI("搬运货物：",num);
            return num;
            // LOGI("now_capacity after：",this->now_capacity);
        }
        else
        {
            now_capacity = 0;
            return now_capacity;
        }
    }

    // 打印信息
    void info()
    {
        LOGI("船只", id, ",状态", state, ",装货量：", capacity, ",剩余容量：", now_capacity, ",剩余容量比例：", now_capacity * 1.0 / capacity, ",泊位id：", berthId, ";");
    }

    float capacityScale(){
        return 1.0 * now_capacity / capacity;
    }
};