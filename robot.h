#pragma once
#include <string>

#include "goods.h"
#include "map.h"
#include "utils.h"
#include "pathFinder.h"
#include "assert.h"
#include "log.h"

enum RobotStatus
{
    IDLE,
    MOVING_TO_GOODS,
    PICKING_UP,
    MOVING_TO_BERTH,
    UNLOADING,
    DEATH
};

class Robot
{
public:
    // 判题器输入数据
    int id;
    Point2d pos;
    int carryingItem; // 0 表示未携带物品，1 表示携带物品
    int state;        // 0 表示恢复状态，1 表示正常运行状态
public:
    RobotStatus status;
    int carryingItemId; // 携带的物品id
    int targetid;
    std::vector<Point2d> path; // 机器人即将要走的路径
private:
    DStarPathfinder pathFinder; // 每个机器人都要存储寻路状态

public:
    Robot(int id, Point2d pos)
        : id(id),
          pos(pos),
          carryingItem(0),
          state(0),
          status(IDLE),
          targetid(0)
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

    std::string move(const Point2d &nextPos)
    {
        std::string instruction;
        if (nextPos.x > pos.x)
            instruction = move(3); // 向下
        else if (nextPos.x < pos.x)
            instruction = move(2); // 向上
        else if (nextPos.y > pos.y)
            instruction = move(0); // 向右
        else if (nextPos.y < pos.y)
            instruction = move(1); // 向左
        return instruction;
    }

    std::string moveWithPath()
    {
        if (!path.empty())
        {
            // A* 算法输出的路径是逆序存储的，以提高弹出效率
            Point2d pos = this->path.back();
            this->path.pop_back();
            return move(pos);
        }
        return std::string("");
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

    bool findPath(const Point2d &destination, const Map &map)
    {
        path = pathFinder.plan(this->pos, destination, map);
        if (!path.empty())
            return true;
        else
            return false;
    }

    /***
     * @brief 路径被障碍物阻挡时，重新寻路
     * @param changedStates 移动障碍物的坐标
    */
    bool refindPath(const Map &map, const std::vector<Point2d> &changedStates)
    {
        path = pathFinder.replan(this->pos, map, changedStates);
        if (!path.empty())
            return true;
        else
            return false;
    }

    // Point2d getPosition() const;
    // bool isCarryingItem() const;
};
