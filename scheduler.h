#pragma once
#include <vector>
#include "goods.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "berth.h"
#include "commandManager.h"

class Scheduler
{
public:
    virtual void scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, CommandManager &commandManager) = 0;
    virtual void scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths, CommandManager &commandManager) = 0;
};

class SimpleTransportStrategy : public Scheduler
{
public:
    void scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, CommandManager &commandManager);
    void scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths, CommandManager &commandManager);
};

// class EfficientTransportStrategy : public Scheduler {
// public:
//     virtual void scheduleRobots(std::vector<Robot>& robots, const Map& map, std::vector<Goods>& goods) = 0;
//     virtual void scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths) = 0;
// };
