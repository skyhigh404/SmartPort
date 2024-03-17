#include "scheduler.h"
#include "pathFinder.h"
#include "log.h"

using std::vector;

int dist(Point2d a, Point2d b) {return abs(a.x-b.x)+abs(a.y-b.y);}

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    // LOGI("test");
    // 寻路
    AStarPathfinder pathfinder;
    vector<vector<std::variant<Path, PathfindingFailureReason>>> path2goods(robots.size(), vector<std::variant<Path, PathfindingFailureReason>>(goods.size(), std::variant<Path, PathfindingFailureReason>())), \
                                    path2berths(robots.size(), vector<std::variant<Path, PathfindingFailureReason>>(berths.size(), std::variant<Path, PathfindingFailureReason>())); //机器人到达货物\泊位的路径
    vector<vector<int>> cost2goods(robots.size(), vector<int>(goods.size())), \
                        cost2berths(goods.size(), vector<int>(berths.size()));
    for (int i=0;i<robots.size();i++) {
        if (robots[i].carryingItem) {
            continue;

        }
        for (int j=0;j<goods.size();j++) {
            path2goods[i][j] = pathfinder.findPath(robots[i].pos, goods[j].pos, map);
            if (std::holds_alternative<Path>(path2goods[i][j]))
                cost2goods[i][j] = std::get<Path>(path2goods[i][j]).size();
            else {
                cost2goods[i][j] = INT_MAX / 2;
                // LOGI("r-g找不到路", i, ' ', j);
            }
            // LOGI("机器人",i,"到货物",j,"的路径长度为：",cost2goods[i][j]);
        }
    }
    for (int i=0;i<goods.size();i++) {
        for (int j=0;j<berths.size();j++) {
            const std::vector<std::vector<int>> &bfsmap = map.berthDistanceMap.at(berths[j].id);
            cost2berths[i][j] = bfsmap[goods[i].pos.x][goods[i].pos.y];
            // LOGI("货物",i,"到泊位",j,"的路径长度为：",cost2berths[i][j]);
        }
    }
    LOGI("test");

    // 衡量收益
    // 计算每个机器人将货物送达泊位的耗时
    std::vector<std::vector<float>> profits(robots.size(), std::vector<float>(goods.size(), 0));
    vector<int> bestBerthIndex(berths.size(), -1);
    for (int i = 0; i < robots.size(); i++) {
        for (int j = 0; j < goods.size(); j++) {
            if (robots[i].carryingItem && robots[i].carryingItemId != goods[j].id) // j or goods[j].id?
                continue;
            // 计算机器人从当前位置到达货物处取货的耗时
            int timeToGoods = cost2goods[i][j];
            int time = INT_MAX;
            // 计算机器人从货物处取货到达泊位的耗时
            for (int k = 0; k < berths.size(); k++) {
                // 如果机器人无法到达该泊位，则跳过该泊位
                if (cost2berths[i][k] == INT_MAX) {
                    continue;
                }
                if (cost2berths[i][k] < time) {
                    time = cost2berths[i][k];
                    bestBerthIndex[j] = k;
                }
                // 计算总耗时
                int totalTime = timeToGoods + cost2berths[i][k];
                // 计算货物的利润
                int profit = goods[j].value;
                // 计算收益
                profits[i][j] = profit *1.0 / totalTime;
            }
            // for (int j=0;j<goods.size();j++) 
                // LOGI("profits ",i,' ',j," :",profits[i][j]);
        }
    }
    // LOGI("test");

    // 确定\分配机器人目的地
    std::vector<std::pair<int, int>> robotDestinations(robots.size(), std::make_pair(-1, -1)); // (货物索引, 泊位索引)
    std::vector<vector<int>> indices(robots.size(), std::vector<int>(goods.size(), 0));
    std::vector<std::pair<int, Action>> robotActions;
    for (int i = 0; i < robots.size(); ++i) {
        for (int j=0;j<goods.size();j++) {
            indices[i][j] = j;
        }
        std::sort(indices[i].begin(), indices[i].end(), [&](int a, int b) {
            return profits[i][a] > profits[i][b]; // 根据第二个维度进行降序排序
        });
        std::sort(profits[i].begin(), profits[i].end(), [&](int a, int b) {
            return profits[i][a] > profits[i][b]; // 根据第二个维度进行降序排序
        });

        // LOGI(i);

        for (int j = 0; j < goods.size(); ++j) {
            int goodsIndex = indices[i][j];
            int berthsIndex = bestBerthIndex[goodsIndex];
            LOGI("判断前：",robots[i],",泊位位置：",berths[berthsIndex].pos);
            // LOGI(j, goods.size());

            if (robots[i].carryingItem && robots[i].carryingItemId!=j) continue; // 机器人和货物不匹配
            if (!robots[i].carryingItem && goods[goodsIndex].status!=0) continue; //货物不可分配
            if (profits[i][j]<1e-4) continue;

            // 携带货物但离泊位还远
            if (robots[i].status==MOVING_TO_BERTH && dist(robots[i].pos, berths[berthsIndex].pos)>6) {
                // robotDestinations[i] = std::make_pair(goodsIndex, berthsIndex);
                LOGI(robots[i],",泊位位置：",berths[berthsIndex].pos);
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, berths[berthsIndex].pos, berths[berthsIndex].id}));
                break;
            } 
            // 携带货物且离泊位近，开始分配具体放货位置
            else if (robots[i].status==MOVING_TO_BERTH && dist(robots[i].pos, berths[berthsIndex].pos)<=6) {
                Point2d berths_pos=berths[berthsIndex].pos;
                int stockpile=berths[berthsIndex].reached_goods.size();
                LOGI(robots[i],",泊位位置：",Point2d(berths_pos.x+stockpile/4, berths_pos.y+stockpile%4));
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, Point2d(berths_pos.x+stockpile/4, berths_pos.y+stockpile%4), berths[berthsIndex].id}));
                robotActions.push_back(std::make_pair(i, Action{FIND_PATH, Point2d(berths_pos.x+stockpile/4, berths_pos.y+stockpile%4), berths[berthsIndex].id}));
                robots[i].status = UNLOADING;
                break;
            }
            // 即将到达卸货地点，到达并卸货
            else if (robots[i].status==UNLOADING && dist(robots[i].pos, robots[i].path[0])==1) {
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, robots[i].path[0], berths[berthsIndex].id}));
                robotActions.push_back(std::make_pair(i, Action{DROP_OFF_GOODS, robots[i].path[0], berths[berthsIndex].id}));
                goods[goodsIndex].status = 3;
                for (int l=0;l<berths[berthsIndex].unreached_goods.size();l++) {
                    if (berths[berthsIndex].unreached_goods[l].id == goods[goodsIndex].id) 
                        berths[berthsIndex].unreached_goods.erase(berths[berthsIndex].unreached_goods.begin() + l);
                }
                berths[berthsIndex].reached_goods.push_back(goods[goodsIndex]);
                robots[i].status = IDLE;
                break;
            }
            // 如果该货物未被分配，并且该机器人未被分配，则将该机器人分配给该货物和泊位
            else if  (robots[i].status==IDLE && goods[goodsIndex].status==0) {
                // robotDestinations[i] = std::make_pair(goodsIndex, berthsIndex);
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id}));
                robotActions.push_back(std::make_pair(i, Action{FIND_PATH, goods[goodsIndex].pos, goods[goodsIndex].id}));
                pickup[i] = goodsIndex;
                goods[goodsIndex].status = 1;
                berths[berthsIndex].unreached_goods.push_back(goods[goodsIndex]);
                robots[i].status = MOVING_TO_GOODS;
                break;
            }
            // 机器人前往货物地点
            else if (robots[i].status==MOVING_TO_GOODS && dist(robots[i].pos, robots[i].path[0])>1) {
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, robots[i].path[0], pickup[i]})); //goodsid
                break;

            }
            // 机器人即将抵达取货地点，到达并取货
            else if (robots[i].status==MOVING_TO_GOODS && dist(robots[i].pos, robots[i].path[0])==1) {
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, robots[i].path[0], pickup[i]})); //goodsid
                robotActions.push_back(std::make_pair(i, Action{PICK_UP_GOODS, robots[i].path[0], pickup[i]}));
                robotActions.push_back(std::make_pair(i, Action{FIND_PATH, berths[berthsIndex].pos, berths[berthsIndex].id}));
                robots[i].status = MOVING_TO_BERTH;
                robots[i].carryingItem = 1;
                robots[i].carryingItemId = goods[goodsIndex].id;
                goods[goodsIndex].status = 2;
                break;
            }
        }
    }
    return robotActions;
}

void SimpleTransportStrategy::calCostAndBestBerthIndes(const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    // LOGI("cal begin:", bestBerthIndex.size(),' ',cost2berths.size(),' ',goods.size());
    for (int i=bestBerthIndex.size(); i<goods.size(); i++) {
        // LOGI(i);
        vector<int> index(berths.size(), -1);
        vector<int> cost(berths.size(), INT_MAX);
        for (int j=0;j<berths.size();j++) {
            cost[j] = map.berthDistanceMap.at(berths[j].id)[goods[i].pos.x][goods[i].pos.y];
            index[j] = j;
        }
        std::sort(index.begin(), index.end(), [&](int a, int b) {
            return cost[a] < cost[b]; // 根据第二个维度进行降序排序
        });
        cost2berths.push_back(cost);
        bestBerthIndex.push_back(index);
        LOGI(bestBerthIndex.size(),' ',cost2berths.size());
    }
    // LOGI("cal end:",bestBerthIndex.size(),' ',cost2berths.size());
}
int SimpleTransportStrategy::WhereIsRobot(Robot& robot, std::vector<Berth> &berths, const Map &map)
{
    for (Berth& berth : berths) {
        if (map.cost(robot.pos, berth.pos)<=6) return berth.id;
    }
    return -1;
}

Action SimpleTransportStrategy::scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug)
{
    if (debug) LOGI("调度开始:goodsize:",goods.size(),". 机器人id：",robot.id);

    // vector<vector<int>> cost2berths(goods.size(), vector<int>(berths.size()));
    // vector<int> bestBerthIndex(goods.size(), -1);
    // for (int i=0;i<goods.size();i++) {
    //     int time = INT_MAX;
    //     for (int j=0;j<berths.size();j++) {
    //         const std::vector<std::vector<int>> &bfsmap = map.berthDistanceMap.at(berths[j].id);
    //         cost2berths[i][j] = bfsmap[goods[i].pos.x][goods[i].pos.y];
    //         if (cost2berths[i][j] < time) {
    //             time = cost2berths[i][j];
    //             bestBerthIndex[i] = j;
    //         }
    //     }
    // }
    calCostAndBestBerthIndes(map, goods, berths);
    if(debug){LOGI("carry状态：",robot.carryingItem,",carry id：",robot.carryingItemId);}

    if (robot.carryingItem==1) for (int b=0;b<berths.size();b++) {
        if (robot.carryingItem==1 && robot.carryingItemId != -1 && bestBerthIndex[robot.carryingItemId][b]!=-1 && berths[bestBerthIndex[robot.carryingItemId][b]].reached_goods.size()<16) {
            if (debug) LOGI("分配泊位");
            Berth &berth = berths[bestBerthIndex[robot.carryingItemId][b]];
            // if (debug) LOGI("泊位已预定货物数量：", berth.reached_goods.size(), ' ', berth.unreached_goods.size());
            robot.targetid = berth.id;
            Point2d dest(-1,-1);
            int nearest = INT_MAX;
            // 去曼哈顿距离最近且有空的位置
            for (int i=3;i>=0;i--) {
                for (int j=3;j>=0;j--) {
                    if (berth.storageSlots[i][j]==-1 && map.cost(robot.pos, berth.pos)<nearest) {
                        dest = Point2d(berth.pos.x+i, berth.pos.y+j);
                        nearest = map.cost(robot.pos, berth.pos);
                    }
                }
            }
            if (nearest<INT_MAX) {
                if(debug){LOGI("分配泊位位置：", dest);}
                return Action{MOVE_TO_BERTH, dest, berth.id};
            }
            if(debug) LOGI("分配泊位失败");
            return Action{FAIL, Point2d(0,0), 0};
        }
    }
    // todo 临时举措
    // if (robot.carryingItem==1 && bestBerthIndex[robot.carryingItemId]!=-1 && robot.carryingItemId != -1) {
    //     if (debug) LOGI("分配泊位");
    //     Berth &berth = berths[bestBerthIndex[robot.carryingItemId]];
    //     // if (debug) LOGI("泊位已预定货物数量：", berth.reached_goods.size(), ' ', berth.unreached_goods.size());
    //     robot.targetid = berth.id;
    //     Point2d dest(0,0);
    //     // 去曼哈顿距离最近且有空的位置
    //     for (int i=3;i>=0;i--) {
    //         for (int j=3;j>=0;j--) {
    //             if (berth.storageSlots[i][j]==-1) {
    //                 dest = Point2d(berth.pos.x+i, berth.pos.y+j);
    //                 if(debug){LOGI("分配泊位位置：",dest);}
    //                 return Action{MOVE_TO_BERTH, dest, berth.id};
    //                 break;
    //             }
    //         }
    //     }
    //     if(debug) LOGI("分配泊位失败");
    //     return Action{FAIL, Point2d(0,0), 0};
    // }

    if (debug) LOGI("计算到货物路径");
    vector<int> cost2goods(goods.size());
    for (int j=0;j<goods.size();j++) {
        if (goods[j].status!=0) {
            cost2goods[j] = INT_MAX;
            continue;
        }
        int berthid = WhereIsRobot(robot, berths, map);
        if (berthid==-1) cost2goods[j] = map.cost(robot.pos, goods[j].pos);
        else cost2goods[j] = map.berthDistanceMap.at(berthid)[goods[j].pos.x][goods[j].pos.y];
    }
    
    if (debug) LOGI("开始衡量收益");
    std::string profit_output="";
    // 计算机器人将货物送达泊位的耗时
    std::vector<float> profits(goods.size(), 0);
    LOGI(goods.size());
    for (int j = 0; j < goods.size(); j++)  {
        int timeToGoods = cost2goods[j];
        // int timeToBerths = cost2berths[j][bestBerthIndex[goods[j].id]];
        int timeToBerths = cost2berths[j][bestBerthIndex[goods[j].id][0]];
        profit_output += "(timetogood:" + std::to_string(timeToGoods) + ",timetoberths:" + std::to_string(timeToBerths) + ") ";
        
        if (timeToBerths==INT_MAX || timeToGoods==INT_MAX) continue;
        profits[j] = goods[j].value*1.0 / (timeToGoods+timeToBerths);
        // profit_output += "(timetogood:" + std::to_string(timeToGoods) + ",timetoberths:" + std::to_string(timeToBerths) + ",profit:" + std::to_string(profits[j]) + ") ";
        // LOGI("货物",j,"收益为：",profits[j]);
    }
    LOGI(profit_output);
    
    if (debug) LOGI("开始分配货物");

    // 确定\分配机器人目的地
    std::vector<int> indices(goods.size(), 0);
    for (int j=0;j<goods.size();j++) {
        indices[j] = j;
    }
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return profits[a] > profits[b]; // 根据第二个维度进行降序排序
    });
    vector<float> profits_sorted;
    std::string profits_output="";
    for (int i : indices) {
        profits_sorted.push_back(profits[i]);
        profits_output += std::to_string(profits[i]) + " ";
    }
    profits = profits_sorted;
    LOGI(profits_output);

    for (int j = 0; j < goods.size(); ++j) {
        int goodsIndex = indices[j];
        // int berthsIndex = bestBerthIndex[goodsIndex];
        int berthsIndex = bestBerthIndex[goodsIndex][0];
        int timeToGoods = cost2goods[j];
        int timeToBerths = cost2berths[j][bestBerthIndex[goods[j].id][0]];
        // LOGI("货物id：",goods[goodsIndex].id,"货物状态：",goods[goodsIndex].status,"货物收益：",profits[j]);
        if (goods[goodsIndex].status==0 && profits[j]>0 && goods[goodsIndex].TTL+10>=timeToGoods+timeToBerths) {
            LOGI("分配货物",goods[goodsIndex].id,",给机器人：",robot.id,"机器人状态：",robot.state);
            robot.targetid = goods[goodsIndex].id;
            goods[goodsIndex].status = 1;
            return Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id};
        }
    }
    if (debug) LOGI("分配货物失败");
    return Action{FAIL, Point2d(0,0), 0};
}

// 根据时间排序unreachedGood
std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths,std::vector<Goods>& goods,std::vector<Robot> &robots,int currentFrame,bool debug)
{
    // todo 根据路径代价计算未到达货物的收益 + 船只在装货时也可以进行抉择
    countGoodInBerth(robots,berths,goods);
    if(debug){LOGI("货物统计完毕");}

    // 当前剩余操作时间 = 游戏时间 - 当前时间( - 船去虚拟点时间)
    int remainder = 15000 - currentFrame;

    
    // 初始化泊位的货物数量
     for (auto& berth : berths) {
            berth.residue_num = berth.reached_goods.size() + berth.unreached_goods.size();
     }

    // 根据船的状态和泊位收益调度
    std::vector<std::pair<int, Action>> shipActions; 
    std::vector<Ship> freeShips;    // 可调度船只
    for(int i = 0;i < ships.size();i++){
        switch (ships[i].state)
        {
        case 0: // 运输中
            // ships[i].now_capacity = ships[i].capacity;
            break;
        case 1: // 正常状态
            // if(ships[i].berthId != -1 && ships[i].now_capacity <= 1){   
            if(ships[i].berthId != -1 && ships[i].now_capacity <= 1){   
                if(debug){LOGI("装满了，发船（移动）");ships[i].info();}

                // 进行统计
                Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);

                //装满,去往虚拟点
                shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                ships[i].now_capacity = ships[i].capacity;
                ships[i].state = 0;
                ships[i].berthId = -1;
            }
            else if(ships[i].berthId != -1){  
                // 泊位货物为空，进行船只调度
                if (berths[ships[i].berthId].reached_goods.size() == 0){
                    if(debug){LOGI("当前没货,");berths[ships[i].berthId].info();}
                    ActionType action = scheudleNormalShip(ships[i],berths[ships[i].berthId],robots);
                    if(action == MOVE_TO_BERTH && berths[ships[i].berthId].canMoveBerth(remainder)){
                        // 加入船只调度队列
                        LOGI("剩余时间：",remainder,",还可以移动：",berths[ships[i].berthId].canMoveBerth(remainder));
                        ships[i].info();
                        freeShips.push_back(ships[i]);
                    }
                    else if(action == DEPART_BERTH || berths[ships[i].berthId].mustGo(remainder)){
                        if(debug){LOGI("没货，船载到货，发船（移动）");ships[i].info();berths[ships[i].berthId].info();LOGI("剩余时间：",remainder,",必须走了：",berths[ships[i].berthId].mustGo(remainder));}

                        Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                        shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                        // 重置船的容量和状态
                        ships[i].now_capacity = ships[i].capacity;
                        ships[i].state = 0;
                        ships[i].berthId = -1;
                    }
                }
                else{   
                    // 计算泊位的溢出货物量
                    berths[ships[i].berthId].residue_num -= ships[i].now_capacity;
                    int berthId = ships[i].berthId;
                    int shipment = std::min(static_cast<int>(berths[berthId].reached_goods.size()),berths[berthId].velocity);

                    // todo 有bug
                    if(debug){LOGI("装货前------------------");ships[i].info();}
                    int res = ships[i].loadGoods(shipment); // 装货
                    berths[berthId].unloadGoods(res);   // 卸货
                    berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);
                    if(debug){LOGI("装货后------------------");ships[i].info();}
                }

            }
            else{ 
                freeShips.push_back(ships[i]);
            }
            break;

        case 2: // 等待状态
            if(berths[ships[i].berthId].residue_num < 0){
                if(debug){LOGI("泊位没有更多货物待装，等待船只加入分配队列");ships[i].info();berths[ships[i].berthId].info();}
                // todo 待判断
                freeShips.push_back(ships[i]);
            }
            else{
                berths[ships[i].berthId].residue_num -= ships[i].now_capacity;
            }
            break;
        }
    }
    // 计算泊位收益
    calculateBerthIncome(berths);
    // todo:
    // 对泊位进行排序
    vector<Berth> berths_copy(berths);
    std::sort(berths_copy.begin(), berths_copy.end(), []( Berth& a,  Berth& b) {
        return a.totalValue > b.totalValue;
    });

    // 对船只进行排序
    std::sort(freeShips.begin(),freeShips.end(),[&](Ship a, Ship b) {
        if(a.berthId == -1 && b.berthId == -1){
            return a.state > b.state;
        }
        else if(a.berthId == -1){
            return true;
        }
        else if(b.berthId == -1){
            return false;
        }
        else{
            return a.state > b.state;
        }
        });

    // 第一次调度船只，根据容量和泊位溢出货量最大化利益;
    // todo 考虑运输完成船只从虚拟点返回泊位的时间
    for (auto& ship : freeShips) {
        for (auto& berth : berths_copy) {
            // 一个泊位最多三只船
            if (ship.now_capacity <= berth.residue_num && shipNumInBerth(berth,ships) <= 2) {
                // 分配的最优目标是当前泊位，则不移动
                if(ship.berthId == berth.id){
                    break;
                }
                // 如果船的容量小于或等于当前泊位的剩余需求，分配船到这个泊位
                if(debug){LOGI("调度船去新泊位（移动）",berth.id);ship.info();}
                ship.berthId = berth.id;
                shipActions.push_back(std::make_pair(ship.id, Action{MOVE_TO_BERTH,Point2d(),berth.id}));
                berth.residue_num -= ship.now_capacity; // 更新泊位的剩余需求
                break; // 跳出循环，继续为下一艘船分配泊位
            }
        }
    }
    // 第二次调度船只，为剩余船只分配泊位,留有两个空闲船只
    // todo 考虑运输完成船只从虚拟点返回泊位的时间
    for (auto& ship : freeShips) {
        for (auto& berth : berths_copy) {
            // if (ship.berthId == -1 && berth.residue_num > 0 && shipNumInBerth(berth,ships) == 0) {
            if (berth.residue_num > 0 && shipNumInBerth(berth,ships) == 0) {  
                if(debug){LOGI("调度船去新泊位（移动）",berth.id);ship.info();}
                if(ship.berthId == berth.id){
                    break;
                }
                ship.berthId = berth.id; 
                shipActions.push_back(std::make_pair(ship.id, Action{MOVE_TO_BERTH,Point2d(),berth.id}));
                berth.residue_num -= ship.now_capacity; // 更新泊位的剩余需求
                break; // 跳出循环，继续为下一艘船分配泊位
            }
        }
    }

    //检查货物
    if(debug){
        for( Berth& berth : berths){
            if(berth.reached_goods.size() >= 10){
                LOGI("船只调度有问题-------------------------");
                berth.info ();
            //     for( auto& ship : ships){
            //         ship.info();
            //     }
            //     if(debug){
            //     LOGI("收益排序后的泊位：");
            //     for(auto &berth : berths_copy){
            //         berth.info();
            //     }
            // }
            }
        }
    }
    return shipActions;
}

// 获取泊位上船的数量
int SimpleTransportStrategy::shipNumInBerth(const Berth& berth,const std::vector<Ship>& ships)\
{
    int num = 0;
    for(const auto& ship : ships){
        if(ship.berthId == berth.id){
            num++;
        }
    }
    return num;
}

// 遍历机器人，统计每个泊位的unreachGoods
void SimpleTransportStrategy::countGoodInBerth(std::vector<Robot> &robots,std::vector<Berth> &berths,std::vector<Goods> goods)
{
    // LOGI("开始泊位货物统计");
    for(auto &robot : robots)
    {
        // LOGI("机器人数量",robots.size());
        // 此时机器人应该拿着货物
        // todo 权宜之计，targetid有问题，机器人一直没放下货物
        // if(robot.carryingItemId != -1 && robot.targetid != -1 && robot.carryingItem == 1 &&robot.targetid < 10)
        if(robot.carryingItemId != -1 && robot.targetid != -1 && robot.carryingItem == 1 )
        {
            // LOGI("机器人ID：",robot.id,",机器人状态：",robot.carryingItem,",货物id:",robot.carryingItemId,",目的泊位id：",robot.targetid);
            berths[robot.targetid].unreached_goods.push_back(goods[robot.carryingItemId]);
        }
    }
    // LOGI("完成了泊位货物的统计");
}

// 计算泊位的收益，每帧重新计算
void SimpleTransportStrategy::calculateBerthIncome(std::vector<Berth> &berths)
{
    for(auto &berth : berths){
        berth.totalValue = 0;
        for(auto &good : berth.reached_goods){
            berth.totalValue += good.value;
        }
        // todo 根据货物送达时间进行衰减
        for(auto &good : berth.unreached_goods){
            // berth.totalValue += static_cast<int>(good.value / 2);
            berth.totalValue += static_cast<int>(good.value);
        }
        // todo平滑处理
        berth.totalValue = static_cast<int>(berth.totalValue * berth.velocity / berth.time);
    }
}

// 调度state为1的船只
ActionType SimpleTransportStrategy::scheudleNormalShip(Ship &ship,Berth &berth,std::vector<Robot> robots)
{
    int GO_TIME = berth.time * 2 + 2;   //  前往虚拟点时间
    int MOVE_TIME = 500 * 2 + 2;    // 500帧
    int can_go_num = 0; 
    int can_move_num = 0;

    // 如果船剩余容量 <= 20%，直接去虚拟点
    // todo 调参
    float now_proportion = ship.now_capacity * 1.0 /ship.capacity;
    LOGI("剩余货物比例：",now_proportion);
    if(now_proportion <= berth.canGoScale){
        LOGI("船剩余货量比例：",now_proportion);
        return DEPART_BERTH;
    }

    // 计算货物是否离太远
    for(auto &good : berth.unreached_goods){
        // 找到携带该货物的机器人
        for(auto &robot : robots){
            if(robot.carryingItemId == good.id && robot.targetid != -1){
                if(robot.path.size() >= GO_TIME){
                    LOGI("机器人路径代价：",robot.path.size(),"，来回虚拟点的代价：",GO_TIME);
                    can_go_num += 1;
                }
                if(robot.path.size() >= MOVE_TIME && robot.targetid != -1){
                    LOGI("机器人路径代价：",robot.path.size(),"，来回泊位的代价：",GO_TIME);
                    can_move_num += 1;
                }
                break;
            }
        }
    }

    // 优先考虑调度船只去其他泊位
    // todo 调参
    // 剩余容量小于0.2
    if(can_move_num == 0 && now_proportion >= berth.canMoveScale){
        return MOVE_TO_BERTH;
    }

    // if(GO_TIME < MOVE_TIME){
    //     // 优先考虑调度船只去其他泊位
    //     // todo 调参
    //     // 剩余容量小于0.2
    //     if(can_go_num == 0 && now_proportion<= 0.95){
    //         LOGI("期间没有到达货物，船剩余货量比例：",ship.now_capacity/ship.capacity);
    //         return DEPART_BERTH;
    //     }
    //     else if(can_move_num == 0 && now_proportion <= 0.95){
    //         return MOVE_TO_BERTH;
    //     }
    // }
    
    // if(GO_TIME > MOVE_TIME){
    //     // 优先考虑调度船只去其他泊位
    //     // todo 调参
    //     if(can_move_num == 0){
    //         return MOVE_TO_BERTH;
    //     }
    //     else if(can_go_num == 0 && (ship.now_capacity / ship.capacity) <= 0.5){
    //         return DEPART_BERTH;
    //     }
    // }
    // else{
    //     if(GO_TIME < MOVE_TIME){
    //     // 优先考虑调度船只去其他泊位
    //     // todo 调参
    //     if(can_go_num == 0 && (ship.now_capacity / ship.capacity) >= 0.1){
    //         return DEPART_BERTH;
    //     }
    //     else if(can_move_num == 0){
    //         return MOVE_TO_BERTH;
    //     }
    //     }
    // }
    return FAIL;
}