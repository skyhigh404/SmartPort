#pragma once
#include <string>

#include "goods.h"
#include "map.h"
#include "utils.h"
#include "assert.h"

class Robot
{
public:
    int id;
    Point2d pos;
    int carryingItem; // 0 表示未携带物品，1 表示携带物品
    int carryingItemId; // 携带的物品id
    int state;        // 0 表示恢复状态，1 表示正常运行状态

public:
    Robot(int id, Point2d pos)
        : id(id),
          pos(pos),
          carryingItem(0),
          state(0)
    {
    }
    std::string move(int direction)
    {
        // 向特定方向移动
        using namespace std::string_literals;
#ifdef DEBUG
        assert(direction >= 0 && direction <= 3);
#endif
        return "move "s + std::to_string(id) + " "s + std::to_string(direction);
    }

    std::string get()
    {
        // 生成取货指令
        using namespace std::string_literals;
        return "get "s + std::to_string(id);
    }

    std::string pull()
    {
        // 生成放置指令
        using namespace std::string_literals;
        return "pull "s + std::to_string(id);
    }

    // Point2d getPosition() const;
    // bool isCarryingItem() const;
};
