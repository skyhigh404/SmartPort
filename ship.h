#pragma once

#include <string>
#include "utils.h"
#include "log.h"

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

    // 给定船的核心点和朝向，返回该船舶占用空间的矩形的左上角和右下角坐标，船舶大小为2*3
    static inline std::pair<Point2d, Point2d> getShipOccupancyRect(const VectorPosition &e)
    {
        // 核心点坐标和朝向到船占据的矩形的映射表
        static const std::array<std::pair<Point2d, Point2d>, 4> occupancyOffsetTable = {
            {{{0, 0}, {1, 2}},   // EAST: 左上角偏移(0,0)，右下角偏移(1,2)
             {{-1, -2}, {0, 0}}, // WEST: 左上角偏移(-1,-2)，右下角偏移(0,0)
             {{-2, 0}, {0, 1}},  // NORTH: 左上角偏移(-2,0)，右下角偏移(0,1)
             {{0, -1}, {2, 0}}}  // SOUTH: 左上角偏移(0,-1)，右下角偏移(2,0)
        };
        const auto &offsets = occupancyOffsetTable[static_cast<int>(e.direction)];
        Point2d topLeft = {e.pos.x + offsets.first.x, e.pos.y + offsets.first.y};
        Point2d bottomRight = {e.pos.x + offsets.second.x, e.pos.y + offsets.second.y};
        return {topLeft, bottomRight};
    }

    // 返回前进一格后的核心点位置和方向
    static inline VectorPosition moveForward(const VectorPosition &vp)
    {
        // 前进一格后的位置
        static const std::array<Point2d, 4> moveOneStep = {{
            {0, 1},  // 右
            {0, -1}, // 左
            {-1, 0}, // 上
            {1, 0}   // 下
        }};
        return {vp.pos + moveOneStep[static_cast<int>(vp.direction)], vp.direction};
    }

    // 返回逆时针旋转一次后的核心点位置和方向
    static inline VectorPosition anticlockwiseRotation(const VectorPosition &vp)
    {
        // 逆时针旋转一次后的方向
        static const std::array<Direction, 4> directionChange = {
            Direction::NORTH, // 右旋转到上
            Direction::SOUTH, // 左旋转到下
            Direction::WEST,  // 上旋转到左
            Direction::EAST   // 下旋转到右
        };
        // 逆时针旋转一次后核心点的位置
        static const std::array<Point2d, 4> corePositionChange = {{
            {1, 1},   // 右
            {-1, -1}, // 左
            {-1, 1},  // 上
            {1, -1}   // 下
        }};
        return {vp.pos + corePositionChange[static_cast<int>(vp.direction)], directionChange[static_cast<int>(vp.direction)]};
    }

    // 返回顺时针旋转一次后的核心点位置和方向
    static inline VectorPosition clockwiseRotation(const VectorPosition &vp)
    {
        // 顺时针旋转一次后的方向
        static const std::array<Direction, 4> directionChange = {
            Direction::SOUTH, // 右旋转到下
            Direction::NORTH, // 左旋转到上
            Direction::EAST,  // 上旋转到右
            Direction::WEST   // 下旋转到左
        };
        // 顺时针旋转一次后核心点的位置
        static const std::array<Point2d, 4> corePositionChange = {{
            {0, 2},  // 右
            {0, -2}, // 左
            {-2, 0}, // 上
            {2, 0}   // 下
        }};
        return {vp.pos + corePositionChange[static_cast<int>(vp.direction)], directionChange[static_cast<int>(vp.direction)]};
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