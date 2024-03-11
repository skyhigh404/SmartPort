#pragma once
#include <vector>
#include "goods.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "berth.h"

enum ActionType
{
    MOVE_TO_POSITION,
    PICK_UP_GOODS,
    DROP_OFF_GOODS,
    MOVE_TO_BERTH,
    DEPART_BERTH,
    FIND_PATH
};

struct Action
{
    ActionType type;
    Point2d desination; // 用于移动
    int targetId;     // 用于标识具体的货物或泊位，根据上下文决定其含义
};


class Scheduler
{
    // 去哪里
public:
    int pickup[10]; // 机器人要取的货的id
    virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths) = 0;
};

class SimpleTransportStrategy : public Scheduler
{
public:
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override;
    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths) override;
};

// class EfficientTransportStrategy : public Scheduler {
// public:
//     virtual void scheduleRobots(std::vector<Robot>& robots, const Map& map, std::vector<Goods>& goods) = 0;
//     virtual void scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths) = 0;
// };
