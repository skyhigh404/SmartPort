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

    bool enterFinal; // 用於首次判定進入終局時刻

    int pickup[10]; // 机器人要取的货的id
    virtual Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) = 0;
    // virtual std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) = 0;
    virtual std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug=false) = 0;
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

    Scheduler(): cost2berths(),bestBerthIndex(),scheduleResult(),bestValue(0),enterFinal(false) {}
};

class SimpleTransportStrategy : public Scheduler
{
public:

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override;
    void  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override {}
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override {}
    vector<float> BerthsValue(const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);

    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug=false) override;
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

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override { return Action();};
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)override{return std::vector<std::pair<int, Action>>();}
    void  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx)override{}
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx)override{}

    StageType getSchedulerType( )override{
        return StageType::FINAL;
    }

    // 调度船只
    std::vector<std::pair<int, Action>> scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug=false)override;
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
    int allocationBerth(int shipId,std::vector<Ship> & ships,std::vector<Berth> &berths){
        if(ship2Berth[shipId] == -1){
            // 遍历找到空闲的泊位id
            // todo 优化空间：找到货物量和船的capacity相匹配的组合；或者如果有船在泊位外等待，则不让他继续走
            for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
                // 当前有位置则不分配
                if(it->second == -1 && shipNumInBerth(berths[it->first],ships) == 0){
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

    bool inAssignedBerth(int berthId){
        if(berth2Ship.count(berthId)== 0){
            return false;
        }
        else{
            return true;
        }
    }

};


// 聚类版终局调度
class FinalClusterTransportStrategy : public Scheduler
{
private:
    bool hasInit = false;   // 标志是否初始化
    std::unordered_map<int,int> ship2Berth; //船只对应的泊位id
    std::unordered_map<int,int> berth2Ship; //泊位对应的船只id
    std::vector<std::vector<Berth>> clusters;   // 每个簇对应的泊位

    int maxCapacity = -1;
    int minVelocity = INT_MAX;
    int maxTime = -1;
    int maxLoadTime;
    
public:

    FinalClusterTransportStrategy(){
        for(int i =0;i < SHIPNUMS;i++){
            ship2Berth[i] = -1;
        }
    }

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override { return Action();};
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)override{return std::vector<std::pair<int, Action>>();}
    void  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx)override{}
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx)override{}

    StageType getSchedulerType( )override{
        return StageType::FINAL;
    }

    // 调度船只
    std::vector<std::pair<int, Action>> scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug=false)override;
    
    // 依次计算每个类中最优的一个泊位，并存在berth2Ship中
    void calculateBestBerths(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods, vector<vector<int>> bestBerthIndex,Map &map,bool debug=false){
        if(!hasInit){
            // 进行聚类
            LOGI("泊位聚类完毕");
            // 依次对每个类做价值排序，并选出唯一的泊位
            for(auto &berth : berths){
                // if(debug){LOGI("计算泊位收益-----");berth.info();}
                berth.totalValue = 0;
                for(auto & good : berth.reached_goods) berth.totalValue += good.value;
                berth.shipInBerthNum = shipNumInBerth(berth,ships);
            }

            // 遍历货物，找到status = 0 和status = 1的货物，找到离他最近的泊位
            int index = 0;
            while(index < goods.size() && goods[index].TTL != -1 && goods[index].TTL != INT_MAX){index++;}
            for(index;index < bestBerthIndex.size();index++){
                if(goods[index].status == 0 || goods[index].status == 1){
                    berths[bestBerthIndex[index][0]].totalValue += goods[index].value;
                }
            }

            ClusteringBerths(berths,map);
            for(std::vector<Berth> cluster : clusters){
                // 依次对每一类的泊位进行排序
                std::vector<Berth> berths_copy(cluster);
                std::sort(berths_copy.begin(), berths_copy.end(),[](Berth& a,Berth& b){
                    // 如果泊位上有船，则优先级最低
                    // todo 如果一个泊位上货物很多很多，要改成“溢出货量”降序排列
                    if(a.shipInBerthNum != b.shipInBerthNum) return a.shipInBerthNum < b.shipInBerthNum;
                    return a.totalValue > b.totalValue;
                });
                // 选排序第一作为该簇的最终泊位
                berth2Ship[berths_copy[0].id] = -1;
                for(int i = 1;i < berths_copy.size(); i++){
                    // 设置不可用
                    Berth::available_berths[berths_copy[i].id] = false;
                }
            }
            if(debug){
                for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
                    LOGI("终局选定泊位：",it->first,"，船只：",it->second);
                    berths[it->first].info();
                }
            }
            //初始化
            for(auto &ship : ships) maxCapacity = std::max(maxCapacity,ship.capacity);
            for(auto &berth : berths) minVelocity = std::min(minVelocity,berth.velocity),maxTime = std::max(maxTime, berth.time);
            maxLoadTime = maxCapacity / minVelocity;
            hasInit = true;
        }
    }

    // 功能函数
    int shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships)
    {
        int num = 0;
        for(const auto& ship : ships){
            // 即使是去泊位路上，也算是该泊位分配了该船
            if(ship.berthId == berth.id){
                num++;
            }
        }
        return num;
    }

    // 给船分配空闲泊位id并返回
    int allocationBerth(int shipId,std::vector<Ship> & ships,std::vector<Berth> &berths){
        if(ship2Berth[shipId] == -1){
            // 遍历找到空闲的泊位id
            // todo 优化空间：找到货物量和船的capacity相匹配的组合；或者如果有船在泊位外等待，则不让他继续走
            for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
                // 当前有位置则不分配
                if(it->second == -1 && shipNumInBerth(berths[it->first],ships) == 0){
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

    bool inAssignedBerth(int berthId){
        if(berth2Ship.count(berthId)== 0){
            return false;
        }
        else{
            return true;
        }
    }

    vector<vector<int>> inner_dist(vector<Berth> berths, Map &map)
    {
        int n = berths.size();
        vector<vector<int>> grid(n, vector<int>(n));
        for (int i=0;i<n;i++) {
            for (int j=0;j<n;j++) {
                grid[i][j] = map.berthDistanceMap.at(berths[i].id)[berths[j].pos.x][berths[j].pos.y];
            }
        }
        return grid;
    }

    // 找到距离最近的两个聚类
    std::pair<int, int> findClosestClusters(const vector<vector<int>>& distanceMatrix, const vector<bool>& merged) {
        int minDist = INT_MAX;
        std::pair<int, int> closestClusters;
        int N = distanceMatrix.size();

        for (int i = 0; i < N; ++i) {
            if (!merged[i]) {
                for (int j = i + 1; j < N; ++j) {
                    if (!merged[j] && distanceMatrix[i][j] < minDist) {
                        minDist = distanceMatrix[i][j];
                        closestClusters = std::make_pair(i, j);
                    }
                }
            }
        }
        return closestClusters;
    }

// 层次聚类函数
    vector<vector<Berth>> hierarchicalClustering(vector<Berth>& berths, const vector<vector<int>>& distanceMatrix, int numClusters) {
        int N = distanceMatrix.size();
        vector<bool> merged(N, false);
        vector<vector<Berth>> clusters;

        // 初始化每个实体为一个独立的聚类
        for (int i = 0; i < N; ++i) {
            clusters.push_back({berths[i]});
        }

        // 重复合并直到只剩下指定数量的聚类
        // LOGI("合并");
        int numClusters_now = clusters.size();
        while (numClusters_now > numClusters) {
            // LOGI("clusters.size():",clusters.size());
            // 找到距离最近的两个聚类
            std::pair<int, int> closest = findClosestClusters(distanceMatrix, merged);
            int cluster1 = closest.first;
            int cluster2 = closest.second;

            // 合并聚类
            for (Berth entity : clusters[cluster2]) {
                clusters[cluster1].push_back(entity);
            }
            // 标记已合并的聚类
            merged[cluster2] = true;

            // 删除被合并的聚类
            // clusters.erase(clusters.begin() + cluster2);
            numClusters_now--;
        }

        int erased = 0;
        for (int i=0;i<N;i++) {
            if (merged[i]) {
                clusters.erase(clusters.begin() + i - erased);
                erased++;
            }
        }

        return clusters;
    }

    void ClusteringBerths(vector<Berth> &berths, Map &map)
    {
        vector<bool> clustered(berths.size(), false);
        // 连通性聚类
        for (int i=0;i<berths.size();i++) {
            Berth& berth = berths[i];
            if (!clustered[i]) {
                vector<Berth> anotherClass;
                anotherClass.push_back(berths[i]);
                for (int j=i+1;j<berths.size();j++) {
                    if (map.berthDistanceMap.at(i)[berths[j].pos.x][berths[j].pos.y] != INT_MAX) {
                        anotherClass.push_back(berths[j]);
                        clustered[j] = true;
                    }
                }
                clusters.push_back(anotherClass);
            }
        }

        for (int i=0;i<clusters.size();i++) {
            LOGI("class ", i, ' ', clusters[i].size());
            for (int j=0;j<clusters[i].size();j++) {
                LOGI(clusters[i][j].pos);
            }
        }

        // 距离聚类
        while (clusters.size()<5) {
            LOGI(clusters.size());
            vector<vector<vector<int>>> inner_dist_grid(clusters.size());
            int max = 0, argmax = -1;
            // 找类内距最大的类进行拆分
            for (int i=0;i<clusters.size();i++) {
                inner_dist_grid[i] = inner_dist(clusters[i], map);
                int total_inner_dist = 0;
                for (int j=0;j<clusters[i].size();j++) for (int k=0;k<clusters[i].size();k++) total_inner_dist += inner_dist_grid[i][j][k];
                if (total_inner_dist > max) {
                    max = total_inner_dist;
                    argmax = i;
                }
            }

            // LOGI(clusters[argmax].size(), ' ', inner_dist_grid[argmax].size());s
            std::vector<std::vector<Berth>> ret = hierarchicalClustering(clusters[argmax], inner_dist_grid[argmax], 2);
            clusters.erase(clusters.begin() + argmax);
            clusters.push_back(ret[0]);
            clusters.push_back(ret[1]);

            LOGI("split");
            for (int i=0;i<clusters.size();i++) {
                LOGI("class ", i, ' ', clusters[i].size());
                for (int j=0;j<clusters[i].size();j++) {
                    LOGI(clusters[i][j].pos);
                }
            }
        }
    }

};



class ImplicitEnumeration : public Scheduler
{
public:
    int Constraint_max_distance;
    int Constraint_total_distance;
    int Constraint_least_berths;
    int Constraint_danger_TTL;
    double coefficient_profit;
    double coefficient_ttl;
    double coefficient_cost;
    vector<int> leastBerthsIndex;
    vector<bool> dontPick;

    int t_ArriveBeforeTTL;
    int t_CalTargetValue;
    int n_ArriveBeforeTTL;
    int n_CalTargetValue;

    Action scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug=false) override;
    std::vector<std::pair<int, Action>>  scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths) override {std::vector<std::pair<int, Action>> ret; return ret;}
    std::vector<std::pair<int, Action>>  scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug=false) override{std::vector<std::pair<int, Action>> ret; return ret;}

    void scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override;
    void LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx) override;
    bool GoodsPickedOnce(vector<int>& array, std::vector<Goods> &goods);
    bool ArriveBeforeTTL(vector<int>& array, vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);
    bool CloseToGood(Robot& robot, Goods& good, const Map &map, std::vector<Berth> &berths, int dist);
    bool LowTotalCost(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int len);
    void calBerthsHoldingGoods(std::vector<Goods> &goods, std::vector<Berth> &berths);
    bool NotTheLeastBerths(Goods& good);
    void calGoodsValue(std::vector<Goods> &goods, std::vector<Berth> &berths, const Map &map, std::vector<Robot> &robots);
    void calGoodsPriority(std::vector<Goods> &goods, std::vector<Berth> &berths, const Map &map, std::vector<Robot> &robots);


    double CalTargetValue(vector<int>& array, std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths);

    StageType getSchedulerType( )override{
        return StageType::SIMPLE;
    }

    ImplicitEnumeration(): Scheduler(),Constraint_max_distance(200),Constraint_total_distance(150),Constraint_least_berths(1),Constraint_danger_TTL(300),coefficient_profit(2),coefficient_ttl(0.1),coefficient_cost(1) {}

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