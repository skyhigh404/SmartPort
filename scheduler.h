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
    FIND_PATH,
    FAIL,
    CONTINUE
};


struct Action
{
    ActionType type;
    Point2d desination; // 用于移动
    int targetId;     // 用于标识具体的货物或泊位，根据上下文决定其含义
    Action(ActionType type, Point2d desination, int targetId) : type(type), desination(desination), targetId(targetId){}
    Action(ActionType type) : type(type), desination(Point2d(-1,-1)), targetId(-1){}
};




class Scheduler
{
    // 去哪里
public:
    int pickup[10]; // 机器人要取的货的id
    virtual Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,bool debug=false) = 0;
    virtual int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) = 0;
    virtual void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods) = 0;
    virtual void calculateBerthIncome(std::vector<Berth> &berths) = 0;
    virtual ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots) = 0;
};

class SimpleTransportStrategy : public Scheduler
{
public:
    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override;
    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,bool debug=false) override;
    int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) override;
    void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods)override;
    void calculateBerthIncome(std::vector<Berth> &berths) override;
    ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots) override;
};

// class EfficientTransportStrategy : public Scheduler {
// public:
//     virtual void scheduleRobots(std::vector<Robot>& robots, const Map& map, std::vector<Goods>& goods) = 0;
//     virtual void scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths) = 0;
// };
