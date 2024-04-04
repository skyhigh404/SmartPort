#pragma once

#include <string>
#include "utils.h"
#include "log.h"
#include "map.h"
#include "pathFinder.h"
#include "assert.h"


class Ship
{
public:
    int id;
    int goodsCount;               // 携带的货物数量
    VectorPosition shipLocAndDir; //  表示船舶位置和方向
    int state;                    // 0: 正常行驶状态, 1: 恢复状态, 2: 装载状态
    int berthId;                  // 目标泊位 ID
    const int price = 8000;       // 购买价格
public:
    static int capacity; // 船的容量
    // int now_capacity;           // 船的剩余容量
    int remainingTransportTime; // 船到目标泊位的剩余运行时间，在处理每一帧信息时维护
private:
    AStarPathfinder<VectorPosition, Map> pathFinder;

public:
    Ship(int id)
        : id(id),
          state(0),
          berthId(-1),
          remainingTransportTime(0) {}

    // 重置船只到主航道
    std::string dept(){
        #ifdef DEBUG
            // 船不在恢复状态
            assert(state != 1);
        #endif
        return "dept";
    }

    // 重置船只到主航道
    std::string dept(){
        #ifdef DEBUG
            // 船不在恢复状态
            assert(state != 1);
        #endif
        return "berth";
    }

    // 旋转命令
    // 重置船只到主航道
    std::string rot(RotationDirection rotDirection){
        #ifdef DEBUG
            // 船在正常行驶状态
            assert(state == 0);
        #endif
        return "rot " + std::to_string(static_cast<int>(rotDirection));
    }

    // 前进命令
    std::string ship(){
        #ifdef DEBUG
            // 船在正常行驶状态
            assert(state == 0);
        #endif
        return "ship";
    }
    
    // 装货,并返回转货的数量
    int loadGoods(int num)
    {
#ifdef DEBUG
        assert(nowCapacity() >= 0);
#endif
        if (nowCapacity() == 0){
            // 异常情况，满货船舶停滞在泊位
            return 0;
        }
        else if (nowCapacity() >= num){
            return num;
        }
        else{
            return nowCapacity();
        }
    }

    // 打印信息
    void info()
    {
        LOGI("船只", id, ",状态", state, ",装货量：", capacity, ",剩余容量：", nowCapacity(), ",剩余容量比例：", nowCapacity() * 1.0 / capacity, ",泊位id：", berthId, ";");
    }

    float capacityScale()
    {
        return 1.0 * nowCapacity() / capacity;
    }

    // 获取船的剩余容量
    int nowCapacity(){
        return std::max(capacity - goodsCount, -1);
    }
};