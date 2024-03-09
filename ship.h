#pragma once

#include <string>
#include "utils.h"

class Ship
{
public:
    int id;
    int capacity;
    Point2d pos;
    int state;   // 0: 运输中, 1: 正常运行状态即装货状态或运输完成状态, 2: 泊位外等待状态
    int berthId; // 目标泊位 ID

    // 目前没有剩余容量标识
public:
    Ship(int id, int capacity) : id(id), capacity(capacity) {}

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
};