#include "scheduler.h"
#include "pathFinder.h"
#include "log.h"

using std::vector;

int dist(Point2d a, Point2d b) {return abs(a.x-b.x)+abs(a.y-b.y);}

// 需要维护的变量：robots、goods、berths
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
            // LOGI(j, goods.size());

            if (robots[i].carryingItem && robots[i].carryingItemId!=j) continue; // 机器人和货物不匹配
            if (!robots[i].carryingItem && goods[goodsIndex].status!=0) continue; //货物不可分配
            if (profits[i][j]<1e-4) continue;

            // 携带货物但离泊位还远
            if (robots[i].status==MOVING_TO_BERTH && dist(robots[i].pos, berths[berthsIndex].pos)>6) {
                // robotDestinations[i] = std::make_pair(goodsIndex, berthsIndex);
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_BERTH, berths[berthsIndex].pos, berths[berthsIndex].id}));
                break;
            } 
            // 携带货物且离泊位近，开始分配具体放货位置
            else if (robots[i].status==MOVING_TO_BERTH && dist(robots[i].pos, berths[berthsIndex].pos)<=6) {
                Point2d berths_pos=berths[berthsIndex].pos;
                int stockpile=berths[berthsIndex].reached_goods.size();
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
    // LOGI("test");

    // 调度机器人（寻路+碰撞检测+避让+指令生成）
    /*
    for (int i = 0; i < robots.size(); i++) {
        // 获取机器人要前往的目的地
        int goodsIndex = robotDestinations[i].first;
        int berthIndex = robotDestinations[i].second;
        
        // 根据机器人的状态生成不同的动作
        if (robots[i].status == RobotStatus::IDLE) {
            // 如果机器人当前处于空闲状态，那么机器人就前往取货
            robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id}));
            // 更新机器人状态
            robots[i].status = RobotStatus::MOVING_TO_GOODS;
        }
        else if (robots[i].status == RobotStatus::MOVING_TO_GOODS) {
            // 如果机器人正在前往取货的途中，检查是否已经到达目的地
            if (robots[i].pos == goods[goodsIndex].pos) {
                // 如果到达目的地，那么机器人就开始取货
                robotActions.push_back(std::make_pair(i, Action{PICK_UP_GOODS, goods[goodsIndex].pos, goods[goodsIndex].id}));
                // 更新机器人状态
                robots[i].status = RobotStatus::PICKING_UP;
            } else {
                // 如果还没有到达目的地，那么继续前往
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id}));
            }
        }
        else if (robots[i].status == RobotStatus::PICKING_UP) {
            // 如果机器人正在取货，检查是否已经完成取货
            if (robots[i].carryingItem) {
                // 如果已经取货成功，那么机器人就前往泊位送货
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, berths[berthIndex].pos, berths[berthIndex].id}));
                // 更新机器人状态
                robots[i].status = RobotStatus::MOVING_TO_BERTH;
            } else {
                // 如果取货失败，这里可以添加一些错误处理逻辑
            }
        }
        else if (robots[i].status == RobotStatus::MOVING_TO_BERTH) {
            // 如果机器人正在前往泊位的途中，检查是否已经到达目的地
            if (robots[i].pos == berths[berthIndex].pos) {
                // 如果到达泊位，那么机器人就完成任务
                robotActions.push_back(std::make_pair(i, Action{DROP_OFF_GOODS, berths[berthIndex].pos, berths[berthIndex].id}));
                // 更新机器人状态
                robots[i].status = RobotStatus::UNLOADING;
            } else {
                // 如果还没有到达目的地，那么继续前往
                robotActions.push_back(std::make_pair(i, Action{MOVE_TO_POSITION, berths[berthIndex].pos, berths[berthIndex].id}));
            }
        } else if (robots[i].status == RobotStatus::UNLOADING) {
            // 如果机器人正在卸货，等待卸货完成
            // 这里可以添加一些逻辑，例如等待一定的时间后更新机器人状态
        }
    }
    // 避让：多个人导致的（用于找出相关机器人），undo，避让代价+不避让损失：
    */
    return robotActions;
}

Action  SimpleTransportStrategy::scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    // LOGI("调度开始:goodsize:",goods.size());
    // 寻路
    AStarPathfinder pathfinder;
    vector<std::variant<Path, PathfindingFailureReason>> path2goods(goods.size(), std::variant<Path, PathfindingFailureReason>()), \
                                    path2berths(berths.size(), std::variant<Path, PathfindingFailureReason>()); //机器人到达货物\泊位的路径
    vector<int> cost2goods(goods.size());
    vector<vector<int>> cost2berths(goods.size(), vector<int>(berths.size()));
    vector<int> bestBerthIndex(goods.size(), -1);
    for (int i=0;i<goods.size();i++) {
        int time = INT_MAX;
        for (int j=0;j<berths.size();j++) {
            const std::vector<std::vector<int>> &bfsmap = map.berthDistanceMap.at(berths[j].id);
            cost2berths[i][j] = bfsmap[goods[i].pos.x][goods[i].pos.y];
            if (cost2berths[i][j] < time) {
                    time = cost2berths[i][j];
                    bestBerthIndex[i] = j;
            }
            // LOGI("货物",i,"到泊位",j,"的路径长度为：",cost2berths[i][j]);
        }
    }
    // LOGI("test");

    if (robot.carryingItem==1 && bestBerthIndex[robot.carryingItemId]!=-1) {
        // LOGI("分配泊位");
        Berth berth = berths[bestBerthIndex[robot.carryingItemId]];
        int num = berth.reached_goods.size() + berth.unreached_goods.size();
        berth.unreached_goods.push_back(goods[robot.carryingItemId]);
        robot.targetid = berth.id;
        Point2d dest(berth.pos.x+num/4, berth.pos.y+num%4);
        return Action{MOVE_TO_BERTH, dest, berths[bestBerthIndex[robot.carryingItemId]].id};
    }

    // LOGI("计算到货物路径");
    for (int j=0;j<goods.size();j++) {
        if (goods[j].status!=0) {
            cost2goods[j] = INT_MAX;
            continue;
        }
        path2goods[j] = pathfinder.findPath(robot.pos, goods[j].pos, map);
        if (std::holds_alternative<Path>(path2goods[j]))
            cost2goods[j] = std::get<Path>(path2goods[j]).size();
        else {
            cost2goods[j] = INT_MAX;
            // LOGI("r-g找不到路", i, ' ', j);
        }
        // LOGI("机器人",i,"到货物",j,"的路径长度为：",cost2goods[i][j]);
    }
    
    // LOGI("开始衡量收益");
    // 衡量收益
    // 计算每个机器人将货物送达泊位的耗时
    std::vector<float> profits(goods.size(), 0);
    for (int j = 0; j < goods.size(); j++) {
        int timeToGoods = cost2goods[j];
        int timeToBerths = cost2berths[j][bestBerthIndex[goods[j].id]];
        if (timeToBerths==INT_MAX || timeToGoods==INT_MAX) continue;
        profits[j] = goods[j].value*1.0 / (timeToGoods+timeToBerths);
        // LOGI("货物",j,"收益为：",profits[j]);
    }
    
    // LOGI("开始分配货物");

    // 确定\分配机器人目的地
    std::vector<int> indices(goods.size(), 0);
    std::vector<std::pair<int, Action>> robotActions;
    for (int j=0;j<goods.size();j++) {
        indices[j] = j;
    }
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return profits[a] > profits[b]; // 根据第二个维度进行降序排序
    });
    vector<float> profits_sorted;
    for (int i : indices) {
        profits_sorted.push_back(profits[i]);
    }
    profits = profits_sorted;

    for (int j = 0; j < goods.size(); ++j) {
        int goodsIndex = indices[j];
        int berthsIndex = bestBerthIndex[goodsIndex];
        // LOGI("货物id：",goods[goodsIndex].id,"货物状态：",goods[goodsIndex].status,"货物收益：",profits[j]);
        if (goods[goodsIndex].status==0 && profits[j]>0) {
            LOGI("分配货物",goods[goodsIndex].id);
            robot.targetid = goods[goodsIndex].id;
            goods[goodsIndex].status = 1;
            return Action{MOVE_TO_POSITION, goods[goodsIndex].pos, goods[goodsIndex].id};
        }
    }
    // LOGI("分配货物失败");
    return Action{FAIL, Point2d(0,0), 0};
}


// 泊位收益排序函数
// 影响因子1：泊位溢出货物的价值之和进行排序
// 影响因子2: 泊位的效率
// todo根据未到达货物的时间、泊位装载速率、泊位运输时间进行综合排序
void sortBerthsByResiGoodsValue(std::vector<Berth>& berths) {
    // LOGI("进入泊位收益计算");
    std::sort(berths.begin(), berths.end(), []( Berth& a,  Berth& b) {
        int totalValueA = 0;
        int totalValueB = 0;
        // 计算a泊位剩余货物的价值
        for(int i = 1;i <= a.residue_num; i++){
            if(i <= a.unreached_goods.size()){
                totalValueA += a.unreached_goods[a.unreached_goods.size()-i].value;
            }else{
                totalValueA += a.reached_goods[a.reached_goods.size() -  i + a.unreached_goods.size()].value;
            }
        }
        a.totalValue = totalValueA;
        // 计算b泊位剩余货物的价值
        for(int i = 1;i <= b.residue_num; i++){
            if(i <= b.unreached_goods.size()){
                totalValueB += b.unreached_goods[b.unreached_goods.size()-i].value;
            }else{
                totalValueB += b.reached_goods[b.reached_goods.size() -  i + b.unreached_goods.size()].value;
            }
        }
        b.totalValue = totalValueB;
        if(a.residue_num > 0 && b.residue_num >0){
            // todo 平滑化处理（装载速率和运输时间之比）
            return a.totalValue * a.velocity / a.time > b.totalValue * a.velocity / a.time;
        }else{
            return a.residue_num > b.residue_num;
        }
    });
}

// 根据时间排序unreachedGood
std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths)
{
    // todo 根据路径代价计算未到达货物的收益 + 船只在装货时也可以进行抉择

    // 初始化泊位的货物数量
     for (auto& berth : berths) {
            berth.residue_num = berth.reached_goods.size() + berth.unreached_goods.size();
        }

    // 根据船的状态和泊位收益调度
    std::vector<std::pair<int, Action>> shipActions; 
    std::vector<Ship> freeShips;    // 空闲船只
    for(int i = 0;i < ships.size();i++){
        switch (ships[i].state)
        {
        case 0:
            // 状态错误
            assert(ships[i].now_capacity == ships[i].capacity);
            break;
        case 1:
            if(ships[i].now_capacity == 0){ //装满,去往虚拟点
                shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                // 重置船的容量和状态
                berths[ships[i].berthId].assigned_ships = std::min(berths[ships[i].berthId].assigned_ships - 1,0);
                ships[i].now_capacity = ships[i].capacity;
                ships[i].state = 0;
                ships[i].berthId = -1;
            }else if(ships[i].berthId != -1){   //装货
                // 计算泊位的溢出货物量
                berths[ships[i].berthId].residue_num -= ships[i].now_capacity;

                // 无货物装配，发船
                if (berths[ships[i].berthId].residue_num <= 0){
                    shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                    // 重置船的容量和状态
                    berths[ships[i].berthId].assigned_ships = std::min(berths[ships[i].berthId].assigned_ships - 1,0);
                    ships[i].now_capacity = ships[i].capacity;
                    ships[i].state = 0;
                    ships[i].berthId = -1;
                }

                int berthId = ships[i].berthId;
                int shipment = 0;
                if(berths[berthId].reached_goods.size() >= berths[berthId].velocity){
                    //堆积货物大于装货速度
                    shipment = berths[berthId].velocity;
                }
                else{
                    shipment = berths[berthId].reached_goods.size();
                }

                int res = ships[i].load(shipment);
                assert(res < berths[berthId].reached_goods.size());
                berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);

            }else if(ships[i].berthId == -1){  // 加入空闲船只列表
                freeShips.push_back(ships[i]);
            }
            break;

        case 2:
            // 等待状态不做改变
            
            // 计算泊位溢出货物量
            berths[ships[i].berthId].residue_num -= ships[i].now_capacity;
            break;
        }
    }

    // 计算泊位的收益并排序
    sortBerthsByResiGoodsValue(berths);
    // 第一次调度船只，根据容量和泊位溢出货量最大化利益
    for (auto& ship : freeShips) {
        ship.info();
        for (auto& berth : berths) {
            berth.info();
            if (ship.capacity <= berth.residue_num) {
                // 如果船的容量小于或等于当前泊位的剩余需求，分配船到这个泊位
                ship.berthId = berth.id;
                shipActions.push_back(std::make_pair(ship.id, Action{MOVE_TO_BERTH,Point2d(),berth.id}));
                // LOGI("分配船",ship.id,"前往泊位",berth.id);
                berth.assigned_ships += 1;
                berth.residue_num -= ship.capacity; // 更新泊位的剩余需求
                break; // 跳出循环，继续为下一艘船分配泊位
            }
        }
    }
    // 第二次调度船只，为剩余船只分配泊位,留有两个空闲船只
    for (auto& ship : freeShips) {
        ship.info();
        for (auto& berth : berths) {
            // LOGI("------------------------再次分配---------------------------------");
            berth.info();
            if (ship.berthId == -1 && berth.residue_num > 0 && berth.assigned_ships == 0) {
                ship.berthId = berth.id;
                shipActions.push_back(std::make_pair(ship.id, Action{MOVE_TO_BERTH,Point2d(),berth.id}));
                berth.assigned_ships += 1;
                // LOGI("第二次分配船",ship.id,"前往泊位",berth.id);
                berth.residue_num -= ship.capacity; // 更新泊位的剩余需求
                break; // 跳出循环，继续为下一艘船分配泊位
            }
        }
    }
    return shipActions;
}