#pragma once
#include <vector>
#include "goods.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "berth.h"
#include "utils.h"



using std::vector;

enum StageType
{
    SIMPLE,
    // FINAL_READY,
    FINAL
};

class Scheduler
{
    // 去哪里
public:
    int pickup[10]; // 机器人要取的货的id
    virtual Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) = 0;
    // virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,int currentFrame,bool debug=false) = 0;
    // virtual int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) = 0;
    // virtual void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods) = 0;
    // virtual void calculateBerthIncome(std::vector<Berth> &berths) = 0;
    // virtual ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots) = 0;
    virtual StageType getSchedulerType() = 0;
};

class SimpleTransportStrategy : public Scheduler
{
private:
    vector<vector<int>> cost2berths; // (gs,bs)
    vector<vector<int>> bestBerthIndex; // (gs,bs)
public:

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    void calCostAndBestBerthIndes(const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    int WhereIsRobot(Robot& robot, std::vector<Berth> &berths, const Map &map);

    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,int currentFrame,bool debug=false) override;
    int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) ;
    void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods);
    void calculateBerthIncome(std::vector<Berth> &berths) ;
    ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots);

    SimpleTransportStrategy(): cost2berths(),bestBerthIndex() {}

    StageType getSchedulerType( )override{
        return StageType::SIMPLE;
    }
};

class FinalTransportStrategy : public Scheduler
{
private:
    bool hasInit = false;   // 标志是否初始化
    std::unordered_map<int,int> ship2Berth; //船只对应的泊位id
    std::unordered_map<int,int> berth2Ship; //泊位对应的船只id

    int maxCapacity = -1;
    int minVelocity = INT_MAX;
    int maxTime = -1;
    int maxLoadTime;
    
public:

    FinalTransportStrategy(){
        for(int i =0;i < SHIPNUMS;i++){
            ship2Berth[i] = -1;
        }
    }

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;

    StageType getSchedulerType( )override{
        return StageType::FINAL;
    }

    // 调度船只
    std::vector<std::pair<int, Action>> scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,int currentFrame,bool debug=false)override;
    // 计算最优的五个泊位，并给对应的availiable_berths赋值
    void calculateBestBerths(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods, vector<vector<int>> bestBerthIndex,bool debug=false);

    // 功能函数
    int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships)
    {
        int num = 0;
        for(const auto& ship : ships){
            if(ship.berthId == berth.id){
                num++;
            }
        }
        return num;
    }

    // 给船分配空闲泊位id并返回
    int allocationBerth(int shipId){
        if(ship2Berth[shipId] == -1){
            // 遍历找到空闲的泊位id
            // todo 优化空间：找到货物量和船的capacity相匹配的组合
            for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
                if(it->second == -1){
                    berth2Ship[it->first] = shipId;
                    ship2Berth[shipId] = it->first;
                    return ship2Berth[shipId];
                }
            }
        }
        else{
            return ship2Berth[shipId];
    }


};
};

// class EfficientTransportStrategy : public Scheduler {
// public:
//     virtual void scheduleRobots(std::vector<Robot>& robots, const Map& map, std::vector<Goods>& goods) = 0;
//     virtual void scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths) = 0;
// };

// class FinalReadyTransportStrategy : public Scheduler
// {
// private:
    
// public:

//     Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;

//     StageType getSchedulerType( )override{
//         return StageType::FINAL_READY;
//     }

//     // 调度船只
//     std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,int currentFrame,bool debug=false) override;
// }