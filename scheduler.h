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
    // virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,int currentFrame,bool debug=false) = 0;
    // virtual int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) = 0;
    // virtual void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods) = 0;
    // virtual void calculateBerthIncome(std::vector<Berth> &berths) = 0;
    // virtual ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots) = 0;
    virtual StageType getSchedulerType() = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    
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
    

    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,int currentFrame,bool debug=false) override;
    int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships) ;
    void countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods);
    void calculateBerthIncome(std::vector<Berth> &berths) ;
    ActionType scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots);

    SimpleTransportStrategy(): Scheduler() {}

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
            assert(ship2Berth[shipId] != -1);
            return -1;
        }
        else{
            return ship2Berth[shipId];
        }
    }


};



class ImplicitEnumeration : public Scheduler
{
public:
    int Constraint_max_distance;
    int Constraint_total_distance;
    int Constraint_least_berths;
    vector<int> leastBerthsIndex;

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override {}
    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,int currentFrame,bool debug=false) override{}

    void scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override;
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override;
    bool GoodsPickedOnce(vector<int>& array, std::vector<Goods> &goods);
    bool ArriveBeforeTTL(vector<int>& array, vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    bool CloseToGood(Robot& robot, Goods& good, const Map &map, std::vector<Berth> &berths, int dist);
    bool ImplicitEnumeration::LowTotalCost(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int len);
    void ImplicitEnumeration::calBerthsHoldingGoods(std::vector<Goods> &goods, std::vector<Berth> &berths);
    bool ImplicitEnumeration::NotTheLeastBerths(Goods& good);



    double CalTargetValue(vector<int>& array, std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);

    StageType getSchedulerType( )override{
        return StageType::SIMPLE;
    }

    ImplicitEnumeration(): Scheduler(),Constraint_max_distance(200),Constraint_total_distance(150),Constraint_least_berths(3) {}

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