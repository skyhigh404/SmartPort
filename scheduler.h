#pragma once
#include <vector>
#include "goods.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "berth.h"
#include "utils.h"



using std::vector;

class Scheduler
{
private:
public:
    vector<vector<int>> cost2berths; // (gs,bs)
    vector<vector<int>> bestBerthIndex; // (gs,bs)
    // 去哪里
    vector<bool> picked;
    vector<int> scheduleResult;
    double bestValue;

    int pickup[10]; // 机器人要取的货的id
    virtual Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,int currentFrame,bool debug=false) = 0;
    
    virtual void  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) = 0;
    virtual void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) = 0;
    void calCostAndBestBerthIndes(const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    vector<int> getResult() {return scheduleResult;}

    Scheduler(): cost2berths(),bestBerthIndex(),scheduleResult(),bestValue(0) {}
};

class SimpleTransportStrategy : public Scheduler
{
public:

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override;
    void  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override {}
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override {}
    

    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,int currentFrame,bool debug=false) override;
    int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) ;
    void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods);
    void calculateBerthIncome(std::vector<Berth> &berths) ;
    ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots);

    SimpleTransportStrategy(): Scheduler() {}
};

class ImplicitEnumeration : public Scheduler
{
public:
    int Constraint_distance;

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override {}
    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,int currentFrame,bool debug=false) override{}

    void scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override;
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override;
    bool GoodsPickedOnce(vector<int>& array, std::vector<Goods> &goods);
    bool ArriveBeforeTTL(vector<int>& array, vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    bool CloseToGood(Robot& robot, Goods& good, const Map &map, std::vector<Berth> &berths, int dist);
    double CalTargetValue(vector<int>& array, std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    
    ImplicitEnumeration(): Scheduler(),Constraint_distance(100) {}
};

// class EfficientTransportStrategy : public Scheduler {
// public:
//     virtual void scheduleRobots(std::vector<Robot>& robots, const Map& map, std::vector<Goods>& goods) = 0;
//     virtual void scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths) = 0;
// };
