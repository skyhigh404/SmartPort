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
          state(-1),
          berthId(-1),
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

    float capacityScale()
    {
        return 1.0 * now_capacity / capacity;
    }
};