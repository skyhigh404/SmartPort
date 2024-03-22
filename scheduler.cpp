#include "scheduler.h"
#include "pathFinder.h"
#include "log.h"

using std::vector;
// Point2d computeCentroid(const std::vector<Berth>& cluster) {
//     Point2d centroid = Point2d(0, 0);
//     for (const Berth& berth : cluster) {
//         centroid.x += berth.pos.x;
//         centroid.y += berth.pos.y;
//     }
//     centroid.x /= cluster.size();
//     centroid.y /= cluster.size();
//     return centroid;
// }

// std::vector<std::vector<Berth>> kMeans(const std::vector<Berth>& berths, Map &map, int k, int maxIterations) {
//     std::vector<Point2d> centroids;
//     std::vector<std::vector<Berth>> clusters(k);

//     // 初始化聚类中心点
//     for (int i = 0; i < k; ++i) {
//         centroids.push_back(berths[i].pos);
//     }

//     // 迭代更新聚类
//     for (int iter = 0; iter < maxIterations; ++iter) {
//         // 清空聚类
//         for (auto& cluster : clusters) {
//             cluster.clear();
//         }

//         // 将每个点分配到最近的聚类
//         for (const Berth& berth : berths) {
//             int closestCentroidIdx = 0;
//             int minDistance = map.berthDistanceMap.at(berth.id)[centroids[0].x][centroids[0].y];
//             for (int i = 1; i < k; ++i) {
//                 int d = map.berthDistanceMap.at(berth.id)[centroids[i].x][centroids[i].y];
//                 if (d < minDistance) {
//                     minDistance = d;
//                     closestCentroidIdx = i;
//                 }
//             }
//             clusters[closestCentroidIdx].push_back(berth);
//         }

//         // 更新聚类中心点
//         for (int i = 0; i < k; ++i) {
//             centroids[i] = computeCentroid(clusters[i]);
//         }
//     }

//     return clusters;
// }

// vector<vector<Berth>> ClusteringBerths(vector<Berth> &berths, Map &map)
// {
//     return kMeans(berths, map, 5, 10);
//     vector<vector<Berth>> result;
//     vector<bool> clustered(berths.size(), false);
//     // 连通性聚类
//     for (int i=0;i<berths.size();i++) {
//         Berth& berth = berths[i];
//         if (!clustered[i]) {
//             vector<Berth> anotherClass;
//             for (int j=i+1;j<berths.size();j++) {
//                 if (map.berthDistanceMap.at(i)[berth.pos.x][berth.pos.y] != INT_MAX) {
//                     anotherClass.push_back(berths[j]);
//                     clustered[j] = true;
//                 }
//             }
//             result.push_back(anotherClass);
//         }
//     }

//     int class_num = result.size();
//     // 距离聚类
//     if (class_num==5) return result; // 正好5（=船数量）类
//     else if (class_num<5) {
//         int max = 0, argmax = -1;
//         // 找类内距最大的类进行拆分
//         for (int i=0;i<result.size();i++) if (result[i].size()>max) {max=result[i].size(); argmax=i;}

//     }
// }

int dist(Point2d a, Point2d b) {return abs(a.x-b.x)+abs(a.y-b.y);}
int WhereIsRobot(Robot& robot, std::vector<Berth> &berths, const Map &map)
{
    for (Berth& berth : berths) {
        if (map.cost(robot.pos, berth.pos)<=6) return berth.id;
    }
    return -1;
}
int TimeToGood(Robot& robot, Goods& good, const Map &map, std::vector<Berth> &berths)
{
    int berthid = WhereIsRobot(robot, berths, map), timeToGood=INT_MAX;
    if (berthid==-1) timeToGood = map.cost(robot.pos, good.pos);
        else timeToGood = map.berthDistanceMap.at(berthid)[good.pos.x][good.pos.y];
    return timeToGood;
}

// 每个货物只由一个机器人取
bool ImplicitEnumeration::GoodsPickedOnce(vector<int>& array, std::vector<Goods> &goods)
{
    vector<bool> picked(goods.size(), false);
    for (int x : array) {
        if (x==-1) continue;
        if (picked[x]) return false;
        picked[x] = true;
    }
    return true;
}
// 取不到货
bool ImplicitEnumeration::ArriveBeforeTTL(vector<int>& array, vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    for (int i=0; i<array.size(); i++) {
        int index = array[i];
        if (index==-1) continue;

        // int berthid = WhereIsRobot(robots[i], berths, map), timeToGood=INT_MAX;
        // if (berthid==-1) timeToGood = map.cost(robots[i].pos, goods[array[i]].pos);
        // else timeToGood = map.berthDistanceMap.at(berthid)[goods[array[i]].pos.x][goods[array[i]].pos.y];
        int timeToGood = TimeToGood(robots[i], goods[index], map, berths);
        
        if (timeToGood + 10 > goods[array[i]].TTL) return false;
    }
    return true;
}
bool ImplicitEnumeration::CloseToGood(Robot& robot, Goods& good, const Map &map, std::vector<Berth> &berths, int dist)
{
    if (TimeToGood(robot, good, map, berths) <= dist) return true;
    return false;
}

bool ImplicitEnumeration::LowTotalCost(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int len)
{
    long long totalTime = 0;
    for (int i=0; i<len; i++) {
        int index = array[i];
        int timeToGood = TimeToGood(robots[i], goods[index], map, berths);
        totalTime += timeToGood;
    }
    if (totalTime< Constraint_total_distance*robots.size()) return true;
    else return false;
}
void ImplicitEnumeration::calBerthsHoldingGoods(std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    vector<int> BerthsHoldingGoods(berths.size(), 0);
    for (int i=0;i<goods.size();i++) {
        Goods& good = goods[i];
        BerthsHoldingGoods[bestBerthIndex[good.id][0]]++;
    }
    leastBerthsIndex = vector<int>(berths.size());
    for (int i=0;i<berths.size();i++) leastBerthsIndex[i]=i;
    std::sort(leastBerthsIndex.begin(), leastBerthsIndex.end(), [&](int a, int b) {
        return BerthsHoldingGoods[a] < BerthsHoldingGoods[b]; // 升序排序
    });
    int index = 0;
    for (Goods& good : goods) {
        for (int i=0;i<Constraint_least_berths;i++) {
            if (bestBerthIndex[good.id][0]==leastBerthsIndex[i]) {
                dontPick[index] = true;
            }
        }
        index++;
    }
}
bool ImplicitEnumeration::NotTheLeastBerths(Goods& good) 
{
    for (int i=0;i<Constraint_least_berths;i++) {
        if (bestBerthIndex[good.id][0]==leastBerthsIndex[i]) return false;
    }
    return true;
}
void ImplicitEnumeration::calGoodsValue(std::vector<Goods> &goods, std::vector<Berth> &berths, const Map &map, std::vector<Robot> &robots)
{
    vector<double> goodsValue(goods.size(), 0);
    for (int i =0;i<goods.size();i++) {
        Goods& good = goods[i];
        goodsValue[i] = good.value / map.berthDistanceMap.at(bestBerthIndex[good.id][0])[good.pos.x][good.pos.y];
    }
    vector<int> goodsIndex(goods.size(), 0);
    for (int i=0;i<goods.size();i++) goodsIndex[i]=i;
    std::sort(goodsIndex.begin(), goodsIndex.end(), [&](int a, int b) {
        return goodsValue[a] < goodsValue[b];
    });
    double valueBound = goodsValue[goodsIndex[goods.size()/2]];
    if (robots.size()>goods.size()/2) valueBound = goodsValue[goodsIndex[0]];
    // double valueBound = std::min(goodsValue[goodsIndex[goods.size()/2]], goodsValue[std::min(robots.size()*2, goods.size()-1)]);
    for (int i=0;i<goods.size();i++) {
        if (goodsValue[i] < valueBound) dontPick[i] = true;
    }
}
void ImplicitEnumeration::calGoodsPriority(std::vector<Goods> &goods, std::vector<Berth> &berths, const Map &map, std::vector<Robot> &robots)
{
    vector<double> goodsPriority(goods.size(), 0);
    for (int i =0;i<goods.size();i++) {
        Goods& good = goods[i];
        if (good.TTL<=0) {goodsPriority[i]=0; continue;}
        goodsPriority[i] = good.value / good.TTL;
    }
    vector<int> goodsIndex(goods.size(), 0);
    for (int i=0;i<goods.size();i++) goodsIndex[i]=i;
    std::sort(goodsIndex.begin(), goodsIndex.end(), [&](int a, int b) {
        return goodsPriority[a] < goodsPriority[b]; // 升序排序
    });
    double priorityBound = goodsPriority[goodsIndex[goods.size()/2]];
    if (robots.size()>goods.size()/2) priorityBound = goodsPriority[goodsIndex[0]];
    // double priorityBound = std::min(goodsPriority[goodsIndex[goods.size()/2]], goodsPriority[std::min(robots.size()*2, goods.size()-1)]); //有问题 to do
    for (int i=0;i<goods.size();i++) {
        if (goodsPriority[i] < priorityBound) dontPick[i] = true;
    }
}
bool AtLeastOne(vector<int>& array)
{
    for (int x:array) if (x!=-1) return true;
    return false;
}
// 计算目标函数值
double ImplicitEnumeration::CalTargetValue(vector<int>& array, std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    long long profit = 0;
    long long cost = 0;
    long long sum_TTL = 0;
    long long profit_TTL = 0;
    long long profit_total = 0;
    for (int i=0;i<array.size();i++) {
        int index = array[i];
        if (index==-1) continue;
        int berthid = WhereIsRobot(robots[i], berths, map), timeToGood=INT_MAX;
        if (berthid==-1) timeToGood = map.cost(robots[i].pos, goods[index].pos);
        else timeToGood = map.berthDistanceMap.at(berthid)[goods[array[i]].pos.x][goods[array[i]].pos.y];
        int timeToBerth = cost2berths[goods[index].id][0];

        profit += goods[index].value;
        cost += timeToGood + timeToBerth;
        sum_TTL += goods[index].TTL;
    }
    for (int i=0;i<goods.size();i++) {
        if (goods[i].TTL < Constraint_danger_TTL && !picked[i]) profit_TTL+=goods[i].value;
        profit_total += goods[i].value;
    }
    LOGI(profit, ' ',cost,' ',sum_TTL);
    int count = 0;
    for (int i=0;i<array.size();i++) if (array[i]==-1) count++;
    if (count==array.size()) cost = -LONG_MAX;
    // 可添加系数
    return coefficient_profit*profit - coefficient_ttl*profit_TTL*1.0 - coefficient_cost*cost;
    return profit*1.0/cost;
    return 1000*profit*2.0 / cost / sum_TTL;
}

// 基于整数规划调度机器人
void ImplicitEnumeration::scheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx)
{
    // 检查约束条件、计算目标值
    if (idx == array.size()) {
        LOGI("check constraint");
        auto start = std::chrono::steady_clock::now();
        if (!ArriveBeforeTTL(array, robots, map, goods, berths)) {
            t_ArriveBeforeTTL += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
            n_ArriveBeforeTTL ++;
            return;
        }
        else {t_ArriveBeforeTTL += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
            n_ArriveBeforeTTL ++;}
        // if (!AtLeastOne(array)) return;
        // LOGI("ArriveBeforeTTL");
        LOGI("calTargetValue");
        start = std::chrono::steady_clock::now();
        double z = CalTargetValue(array, robots, map, goods, berths);
        t_CalTargetValue += std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
        n_CalTargetValue ++;
        if (z > bestValue) {
            std::string output="";
            for (int x:array) {
                output += std::to_string(x) + " ";
            }
            LOGI("find better value:", z, ",result:",output);
            bestValue = z;
            scheduleResult = array;
        }
        return;
    }

    if (goods.size()<robots.size()) {
        array[idx] = -1;
        int count = 0;
        for (int i=0;i<idx;i++) if (array[i]==-1) count++;
        if (count>robots.size()-goods.size()) return;
        scheduleRobots(robots, map, goods, berths, array, idx + 1);
    }
    // 依次尝试将当前位置的值设为取货索引
    for (int i=0; i<goods.size(); i++) {
        // 剪枝
        if (picked[i]) continue;
        if (dontPick[i]) continue;
        // if (!CloseToGood(robots[idx], goods[i], map, berths, Constraint_max_distance)) continue;
        // if (!NotTheLeastBerths(goods[i])) continue;
        // if (!LowTotalCost(robots, map, goods, berths, array, idx)) continue;

        picked[i] = true;
        array[idx] = i;

        // 递归调用下一个位置
        scheduleRobots(robots, map, goods, berths, array, idx + 1);
        picked[i] = false;
    }
}

// 整数规划启动器
void ImplicitEnumeration::LPscheduleRobots(std::vector<Robot> &robots, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, vector<int>& array, int idx)
{
    picked = vector<bool>(goods.size(), false);
    dontPick = vector<bool>(goods.size(), false);
    bestValue = -LONG_MAX;
    scheduleResult.clear();


    t_ArriveBeforeTTL=0;
    t_CalTargetValue=0;
    n_ArriveBeforeTTL=0;
    n_CalTargetValue=0;

    if (goods.size()==0 || robots.size()==0) return;
    calBerthsHoldingGoods(goods, berths);
    calGoodsValue(goods, berths, map, robots);
    calGoodsPriority(goods, berths, map, robots);
    int count = 0;
    for (int i=0;i<goods.size();i++) if (dontPick[i]) count++;
    LOGI("开始LP调度，机器人数：",robots.size(),",货物数：",goods.size(),",可选货物数：",goods.size()-count);

    scheduleRobots(robots, map, goods, berths, array, idx);

    std::string output="";
    for (int x:scheduleResult) {
        output += std::to_string(x) + " ";
    }
    LOGI("调度结果：",output,",目标值：",bestValue);
    LOGI("t_ArriveBeforeTTL:",t_ArriveBeforeTTL,",t_CalTargetValue:",t_CalTargetValue,",n_ArriveBeforeTTL",n_ArriveBeforeTTL,",n_CalTargetValue",n_CalTargetValue);
}

Action ImplicitEnumeration::scheduleRobot(Robot &robot, const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths, bool debug)
{
    if (debug) LOGI("调度开始:goodsize:",goods.size(),". 机器人id：",robot.id);
    calCostAndBestBerthIndes(map, goods, berths);
    if(debug){LOGI("carry状态：",robot.carryingItem,",carry id：",robot.carryingItemId);}

    if (robot.carryingItem==1) for (int b=0;b<berths.size();b++) {
        if (robot.carryingItem==1 && robot.carryingItemId != -1 && bestBerthIndex[robot.carryingItemId][b]!=-1 && berths[bestBerthIndex[robot.carryingItemId][b]].reached_goods.size()<16) {
            if (debug) LOGI("分配泊位");
            Berth &berth = berths[bestBerthIndex[robot.carryingItemId][0]];
            // if (debug) LOGI("泊位已预定货物数量：", berth.reached_goods.size(), ' ', berth.unreached_goods.size());
            robot.targetid = berth.id;
            Point2d dest(-1,-1);
            int nearest = INT_MAX;
            // 去曼哈顿距离最近且有空的位置
            for (int i=3;i>=0;i--) {
                for (int j=3;j>=0;j--) {
                    // if (berth.storageSlots[i][j]==-1 && map.cost(robot.pos, berth.pos)<nearest) {
                    if (map.cost(robot.pos, Point2d(berth.pos.x+i, berth.pos.y+j))<nearest) {
                        dest = Point2d(berth.pos.x+i, berth.pos.y+j);
                        nearest = map.cost(robot.pos, Point2d(berth.pos.x+i, berth.pos.y+j));
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
    return Action{FAIL, Point2d(0,0), 0};
}


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

void Scheduler::calCostAndBestBerthIndes(const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    // LOGI("cal begin:", bestBerthIndex.size(),' ',cost2berths.size(),' ',goods.size());
    LOGI("cal begin");
    for (int i=bestBerthIndex.size(); i<goods.size(); i++) {
        // LOGI("goodsid:",i);
        vector<int> index(berths.size(), -1);
        vector<int> cost(berths.size(), INT_MAX);
        for (int j=0;j<berths.size();j++) {
            if (Berth::available_berths[j]) cost[j] = map.berthDistanceMap.at(berths[j].id)[goods[i].pos.x][goods[i].pos.y];
            else cost[j] = INT_MAX;
            // LOGI(cost[j]);
            index[j] = berths[j].id;
        }
        std::sort(index.begin(), index.end(), [&](int a, int b) {
            return cost[a] < cost[b]; // 根据第二个维度进行降序排序
        });
        cost2berths.push_back(cost);
        bestBerthIndex.push_back(index);
        // LOGI(bestBerthIndex.size(),' ',cost2berths.size());
        LOGI("最佳泊位:", bestBerthIndex[i][0]);
    }
    // LOGI("cal end:",bestBerthIndex.size(),' ',cost2berths.size());
}

// 返回berths.size()+1维的向量，前berths.size()表示各泊位的价值，最后一位表示总价值
vector<float> SimpleTransportStrategy::BerthsValue(const Map &map, std::vector<Goods> &goods, std::vector<Berth> &berths)
{
    vector<float> berthsValue(berths.size(), 0);
    float totalGoodsValue = 0;
    for (auto& good:goods) {
        if (good.status==0) {
            Berth& berth = berths[bestBerthIndex[good.id][0]];
            berthsValue[berth.id] += good.value*1.0 / map.berthDistanceMap.at(berth.id)[good.pos.x][good.pos.y] * good.TTL>500?1.5:1 ;
            totalGoodsValue += good.value*1.0 / map.berthDistanceMap.at(berth.id)[good.pos.x][good.pos.y] * good.TTL>500?1.5:1 ;
        }
    }
    berthsValue.push_back(totalGoodsValue);
    return berthsValue;
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
    
    auto start = std::chrono::steady_clock::now();
    calCostAndBestBerthIndes(map, goods, berths);
    if(debug){LOGI("carry状态：",robot.carryingItem,",carry id：",robot.carryingItemId);}
    auto end = std::chrono::steady_clock::now();
    LOGI("calCostAndBestBerthIndes时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");

    start = std::chrono::steady_clock::now();
    // if (robot.carryingItem==1) for (int b=0;b<berths.size();b++) {
        if (robot.carryingItem==1 && robot.carryingItemId != -1 && bestBerthIndex[robot.carryingItemId][0]!=-1) {
            if (debug) LOGI("分配泊位");
            Berth &berth = berths[bestBerthIndex[robot.carryingItemId][0]];
            // if (debug) LOGI("泊位已预定货物数量：", berth.reached_goods.size(), ' ', berth.unreached_goods.size());
            robot.targetid = berth.id;
            Point2d dest(-1,-1);
            int nearest = INT_MAX;
            // 去曼哈顿距离最近且有空的位置
            for (int i=3;i>=0;i--) {
                for (int j=3;j>=0;j--) {
                    // if (berth.storageSlots[i][j]==-1 && map.cost(robot.pos, berth.pos)<nearest) {
                    if (map.cost(robot.pos, Point2d(berth.pos.x+i, berth.pos.y+j))<nearest) {
                        dest = Point2d(berth.pos.x+i, berth.pos.y+j);
                        nearest = map.cost(robot.pos, Point2d(berth.pos.x+i, berth.pos.y+j));
                    }
                }
            }
            if (nearest<INT_MAX) {
                if(debug){LOGI("分配泊位位置：", dest);}
                return Action{MOVE_TO_BERTH, dest, berth.id};
            }
            if(debug) LOGI("分配泊位失败");
            return Action{FAIL, Point2d(0,0), -1};
        }
    // }
    end = std::chrono::steady_clock::now();
    LOGI("分配泊位时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");
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

    start = std::chrono::steady_clock::now();
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
    end = std::chrono::steady_clock::now();
    LOGI("计算到货物路径时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");
    
    start = std::chrono::steady_clock::now();
    if (debug) LOGI("开始衡量收益");
    std::string profit_output="";
    // 计算机器人将货物送达泊位的耗时
    std::vector<float> profits(goods.size(), 0);
    vector<float> berthsValue = BerthsValue(map, goods, berths);
    // LOGI(goods.size());
    for (int j = 0; j < goods.size(); j++)  {
        long long timeToGoods = cost2goods[j];
        // int timeToBerths = cost2berths[j][bestBerthIndex[goods[j].id]];
        long long timeToBerths = cost2berths[j][bestBerthIndex[goods[j].id][0]];
        // profit_output += "(timetogood:" + std::to_string(timeToGoods) + ",timetoberths:" + std::to_string(timeToBerths) + ") ";
        if (timeToBerths==INT_MAX || timeToGoods==INT_MAX) continue;
        
        bool canReach = false;
        for (int k=0;k<berths.size();k++) {
            // vector<vector<int>> grid = map.berthDistanceMap.at(k);
            if (map.berthDistanceMap.at(k)[robot.pos.x][robot.pos.y]!=INT_MAX && map.berthDistanceMap.at(k)[goods[j].pos.x][goods[j].pos.y]!=INT_MAX) {canReach = true;break;}
        }
        if (!canReach) continue;

        // if (timeToGoods > 200) continue;
        profits[j] = goods[j].value*1.0 / (1*timeToGoods+timeToBerths);
        if (goods[j].TTL<=500 && !enterFinal) profits[j] *= 1.2;
        // if (berthsValue[bestBerthIndex[goods[j].id][0]] > berthsValue[berths.size()]/berths.size() && !enterFinal) profits[j] *= 1.5;

        profit_output += "(timetogood:" + std::to_string(timeToGoods) + ",timetoberths:" + std::to_string(timeToBerths) + ",value:" + std::to_string(goods[j].value) + ",profit:" + std::to_string(profits[j]) + ") ";
        // LOGI("货物",j,"收益为：",profits[j]);
    }
    end = std::chrono::steady_clock::now();
    LOGI("衡量收益时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");
    // LOGI(profit_output);

    // 预估接下来n步的收益
    // int n = 1;
    // std::vector<float> profits_future(goods.size(), 0);
    // for (int i =0 ;i<goods.size();i++) {
    //     if (profits[i]==0) continue;
    //     int berthID = bestBerthIndex[i][0], nextberthID=bestBerthIndex[i][0];
    //     int cost_total = 0;
    //     for (int k=0;k<n;k++) {
    //         float profits_k = 0;
    //         float value_k = 0;
    //         int cost_k = 0;
    //         berthID = nextberthID;
    //         for (int j=0;j<goods.size();j++) {
    //             if (profits[j]==0) continue;
    //             // int timeToGood=cost2berths[j][berthID], timeToBerth=cost2berths[j][bestBerthIndex[j][0]];
    //             float profit_tmp = goods[j].value*1.0 / (cost2berths[j][berthID]+cost2berths[j][bestBerthIndex[j][0]]);
    //             if (profit_tmp > profits_k) {profits_k = profit_tmp;nextberthID=bestBerthIndex[j][0]; value_k = goods[j].value; cost_k=cost2berths[j][berthID]+cost2berths[j][bestBerthIndex[j][0]];}
    //         }
    //         profits_future[i] += value_k;
    //         cost_total += cost_k;
    //     }
    //     profits_future[i] /= cost_total;
    // }
    
    if (debug) LOGI("开始分配货物");

    start = std::chrono::steady_clock::now();
    // 确定\分配机器人目的地
    std::vector<int> indices(goods.size(), 0);
    for (int j=0;j<goods.size();j++) {
        indices[j] = j;
    }
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        // return profits[a]+0.2*profits_future[a] > profits[b]+0.2*profits_future[b]; // 根据第二个维度进行降序排序
        return profits[a] > profits[b]; // 根据第二个维度进行降序排序
    });
    vector<float> profits_sorted;
    std::string profits_output="";
    for (int i : indices) {
        profits_sorted.push_back(profits[i]);
        profits_output += std::to_string(profits[i]) + " ";
    }
    // profits = profits_sorted;
    // LOGI(profits_output);
//robot
    for (int j = 0; j < goods.size(); ++j) {
        int goodsIndex = indices[j];
        Goods& good = goods[goodsIndex];
        // int berthsIndex = bestBerthIndex[goodsIndex];
        int berthsIndex = bestBerthIndex[good.id][0];
        int timeToGoods = cost2goods[good.id];
        int timeToBerths = cost2berths[good.id][bestBerthIndex[good.id][0]];
        if (timeToBerths==INT_MAX || timeToGoods==INT_MAX) continue;
        // LOGI("货物id：",good.id,"货物状态：",good.status,"货物收益：",profits[good.id]);
        if (good.status==0 && profits[good.id]>0 && good.TTL+10>=timeToGoods && berthCluster[berthsIndex]==assignment[robot.id]) {
            LOGI("分配货物",goods[goodsIndex].id,",给机器人：",robot.id,"机器人状态：",robot.state);
            robot.targetid = good.id;
            good.status = 1;
            return Action{MOVE_TO_POSITION, good.pos, good.id};
        }
    }
    end = std::chrono::steady_clock::now();
    LOGI("分配货物时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");
    if (debug) LOGI("分配货物失败");
    return Action{FAIL, Point2d(0,0), 0};
}

// 根据时间排序unreachedGood
std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug)
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
            if(ships[i].berthId != -1){  
                if(berths[ships[i].berthId].mustGo(remainder)){
                    if(debug){LOGI("剩余时间：",remainder,",必须走了：",berths[ships[i].berthId].mustGo(remainder));}
                    Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                    ships[i].goStatus();
                    shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                }
                else if(ships[i].now_capacity <= 1){
                    // 装满则发货
                    if(debug){LOGI("装满了，发船（移动）");ships[i].info();berths[ships[i].berthId].info();}
                    Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                    ships[i].goStatus();
                    //装满,去往虚拟点
                    shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                }
                // 泊位货物为空，进行船只调度
                else if (berths[ships[i].berthId].reached_goods.size() == 0){
                    if(debug){LOGI("当前没货,");berths[ships[i].berthId].info();}
                    ActionType action = scheudleNormalShip(ships[i],berths[ships[i].berthId],robots);
                    if(action == MOVE_TO_BERTH && berths[ships[i].berthId].canMoveBerth(remainder)){
                        // 加入船只调度队列
                        LOGI("剩余时间：",remainder,",还可以移动：",berths[ships[i].berthId].canMoveBerth(remainder));
                        ships[i].info();
                        freeShips.push_back(ships[i]);
                    }
                    // else if(action == DEPART_BERTH || berths[ships[i].berthId].mustGo(remainder)){
                    else if(action == DEPART_BERTH){
                        if(debug){LOGI("没货，船载到货，发船（移动）");ships[i].info();berths[ships[i].berthId].info();}
                        Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                        ships[i].goStatus();
                        shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
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
                    // berths[berthId].unloadGoods(res);   // 卸货
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
    // todo 排序因素调参
    std::sort(freeShips.begin(),freeShips.end(),[&](Ship a, Ship b) {
        if(a.berthId == -1 && b.berthId == -1){
            return a.now_capacity > b.now_capacity;
        }
        else if(a.berthId == -1){
            return true;
        }
        else if(b.berthId == -1){
            return false;
        }
        else{
            return a.now_capacity > b.now_capacity;
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
    // todo 考虑运输完成船只从虚拟点返回泊位的时间，去除前面已经调度的船舶
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
        // todo 可以调参
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
    return FAIL;
}

// 调度船只
std::vector<std::pair<int, Action>>  FinalTransportStrategy::scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug){
    // 设置五个唯一泊位
    calculateBestBerths(ships,berths,goods,bestBerthIndex,debug);
    if(debug) ("分配完泊位");
    int remainder = 15000 - currentFrame;

    // 遍历船，进行调度
    // 如果船已经在预定泊位上，货满则出发 | mustGo(2*time) == false：有货拿货，无货剩余容量<=90%直接走 | mustGo(2*time) == true && mustGo() == false，则直接走；容量>=90%，不移动
    // 如果船已经在其他泊位上，货满则出发 | mustGo(2*time) == true则出发；有货&&mustGo(2*time) == false 则继续装;
    // 如果船在等待状态：货多且mustGo(2 time)，直接出发虚拟点；不然直接去泊位
    // 如果船在虚拟点，直接去泊位
    std::vector<std::pair<int, Action>> shipActions;
    for(int i=0;i < SHIPNUMS;i++){
        float nowCapaityProportion = 1.0 * ships[i].now_capacity / ships[i].capacity;
        switch (ships[i].state)
        {
        case 2:
            // 在泊位外等待，如果当前berth id和分配的id不一致，则调度去指定泊位；否则可以等待
            if(!inAssignedBerth(ships[i].berthId)){
                // 分配指定泊位
                int assignedBerthId = allocationBerth(ships[i].id,ships,berths);
                shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),assignedBerthId}));
            }
            break;
        case 1:
            // 货满则出发
            if (ships[i].berthId != -1 && (ships[i].now_capacity <= 1 || berths[ships[i].berthId].mustGo(remainder))){
                //装满,去往虚拟点
                // 进行统计
                LOGI("货满了或者没时间了，直接去虚拟点：");
                ships[i].info();
                Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                ships[i].goStatus();
                shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
            }
            // 在虚拟点，直接出发
            else if(ships[i].berthId == -1){
                // 分配指定泊位
                int assignedBerthId = allocationBerth(ships[i].id,ships,berths);
                ships[i].berthId = assignedBerthId;
                LOGI("在虚拟点，直接出发前往泊位：",assignedBerthId);
                ships[i].info();
                shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),assignedBerthId}));
            }
            else{
                // 到了终局准备阶段前直接走;一去一回
                // if(berths[ships[i].berthId].mustGo(remainder,2 * berths[ships[i].berthId].time + maxLoadTime) && nowCapaityProportion < 0.5){
                if(berths[ships[i].berthId].mustGo(remainder,2 * maxTime + maxLoadTime) && nowCapaityProportion < 0.5){
                    //装满,去往虚拟点
                    LOGI("进入终局准备阶段，直接去虚拟点：");
                    ships[i].info();
                    berths[ships[i].berthId].info();
                    Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                    ships[i].goStatus();
                    shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                }
                //  货物数量为0
                else if(berths[ships[i].berthId].reached_goods.size() == 0){
                    // // todo 可调参
                    // // 泊位一致
                    // if(inAssignedBerth(ships[i].berthId)){
                    //     //剩余容量 <= 70% ，去虚拟点 
                    //     // todo 可调参
                    //     if(nowCapaityProportion < 0.7){
                    //         Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                    //         ships[i].goStatus();
                    //         shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                    //     }
                    // }
                    // 泊位不一致
                    if(!inAssignedBerth(ships[i].berthId)){
                        // 剩余货量 >= 70%，去对应泊位;否则去虚拟点
                        if(nowCapaityProportion < 0.7){
                            Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                            LOGI("不在预定泊位上，当前剩余容量:",nowCapaityProportion,",去虚拟点");
                            ships[i].info();
                            ships[i].goStatus();
                            shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                        }
                        else{
                            int assignedId = allocationBerth(ships[i].id,ships,berths);
                            LOGI("不在预定泊位上，当前剩余容量:",nowCapaityProportion,",去预定泊位");
                            ships[i].info();
                            ships[i].berthId = assignedId;
                            shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),assignedId}));
                        }
                        berths[ships[i].berthId].info();
                    }
                    
                }
                // 有货拿货
                else{
                    int shipment = std::min(static_cast<int>(berths[ships[i].berthId].reached_goods.size()),berths[ships[i].berthId].velocity);

                    // if(debug){LOGI("装货前------------------");ships[i].info();}
                    int res = ships[i].loadGoods(shipment); // 装货
                    // berths[berthId].unloadGoods(res);   // 卸货
                    berths[ships[i].berthId].reached_goods.erase(berths[ships[i].berthId].reached_goods.begin(),berths[ships[i].berthId].reached_goods.begin() + res);
                    // if(debug){LOGI("装货后------------------");ships[i].info();}
                }
            }
            break;
        case 0:
            break;
        }
    }
    LOGI("成功送达比例：--------------------------------");
    berths[0].info();
    if(debug && currentFrame> 14990&& currentFrame < 14995){
        for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
            LOGI("分配的五个泊位：",it->first,",",it->second);
        }
    }
    return shipActions;
}

// 计算最优的五个泊位，并给对应的availiable_berths赋值
void FinalTransportStrategy::calculateBestBerths(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods, std::vector<vector<int>> bestBerthIndex,bool debug)
{
    if(!hasInit){
        // 初始化泊位价值,计算已到达货物的价值;统计泊位上船的数量
        for(auto &berth : berths){
            // if(debug){LOGI("计算泊位收益-----");berth.info();}
            berth.totalValue = 0;
            for(auto & good : berth.reached_goods) berth.totalValue += good.value;
            berth.shipInBerthNum = shipNumInBerth(berth,ships);
        }

        // 遍历货物，找到status = 0 和status = 1的货物，找到离他最近的泊位
        int index = 0;
        while(index < goods.size() && goods[index].TTL != -1 && goods[index].TTL != INT_MAX){index++;}
        // 执行前调度一次 calCostAndBestBerthIndes 更新bestBerthIndex
        for(index;index < bestBerthIndex.size();index++){
            if(goods[index].status == 0 || goods[index].status == 1){
                // 找到货物最近的泊位，并累加价值
                // LOGI("bestBerthIndex size:",bestBerthIndex.size(),",goods size:",goods.size(),",最佳泊位序号:",bestBerthIndex[index][0]);
                berths[bestBerthIndex[index][0]].totalValue += goods[index].value;
            }
        }
        std::vector<Berth> berths_copy(berths);
        // 对泊位进行排序，选择价值最高的五个泊位
        std::sort(berths_copy.begin(), berths_copy.end(),[](Berth& a,Berth& b){
            // 可调参 todo
            // if(std::abs(a.totalValue - b.totalValue) < 200){
            //     // 差不多价值下，泊位上有船的优先级更高
            //     return a.shipInBerthNum > b.shipInBerthNum;
            // }
            return a.totalValue > b.totalValue;
        });
        // 选择价值最低的五个设置availiable = false，让机器人不往那里运货
        for(int i = SHIPNUMS;i < BERTHNUMS;i++){
            LOGI("禁用泊位id：",berths_copy[i].id);
            Berth::available_berths[berths_copy[i].id] = false;
            // 初始化berth2Ship变量
            berth2Ship[berths_copy[i-SHIPNUMS].id] = -1;
        }
        if(debug){
            LOGI("泊位收益排序：");
            for(auto &berth : berths_copy){
                berth.info();
            }
        }
        //初始化
        for(auto &ship : ships) maxCapacity = std::max(maxCapacity,ship.capacity);
        for(auto &berth : berths) minVelocity = std::min(minVelocity,berth.velocity),maxTime = std::max(maxTime, berth.time);
        maxLoadTime = maxCapacity / minVelocity;
        hasInit = true;
    }
}

// 调度船只
std::vector<std::pair<int, Action>>  FinalClusterTransportStrategy::scheduleShips(std::vector<Ship> &ships, std::vector<Berth> &berths,std::vector<Goods>& goods,std::vector<Robot> &robots,std::vector<vector<int>> bestBerthIndex,Map &map,int currentFrame,bool debug){
    // 聚簇获得五个分配的泊位
    calculateBestBerths(ships,berths,goods,bestBerthIndex,map,debug);
    if(debug) ("分配完泊位");
    int remainder = 15000 - currentFrame;

    std::vector<std::pair<int, Action>> shipActions;
    // 只有游戏快结束时才判断
    // 计算泊位的溢出货物量，如果 < 0，则不让机器人送货过来
    if( remainder >= maxTime && remainder <= maxTime * 2){
        LOGI("判断时间：",15000 - maxTime * 2,"," , 15000 - maxTime);
        for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
            int shipId = berth2Ship[it->first] == -1 ? shipInBerth(berths[it->first],ships) : berth2Ship[it->first];
            if(shipId != -1){
                if(ships[shipId].now_capacity <= berths[it->first].reached_goods.size() && Berth::available_berths[it->first] == true) 
                {
                    Berth::available_berths[it->first] = false;
                    LOGI("当前泊位货物溢出，不用再送货过来：");
                    ships[shipId].info();
                    berths[it->first].info();
                }
            }
        }
    }
    
    for(int i=0;i < SHIPNUMS;i++){
        float nowCapaityProportion = 1.0 * ships[i].now_capacity / ships[i].capacity;
        switch (ships[i].state)
        {
        case 2:
            // 在泊位外等待，如果当前berth id和分配的id不一致，则调度去指定泊位；否则可以等待
            if(!inAssignedBerth(ships[i].berthId)){
                // 分配指定泊位
                int assignedBerthId = allocationBerth(ships[i].id,ships,berths);
                shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),assignedBerthId}));
            }
            break;
        case 1:
            // 货满则出发
            if (ships[i].berthId != -1 && (ships[i].now_capacity <= 0 || berths[ships[i].berthId].mustGo(remainder))){
                //装满,去往虚拟点
                // 进行统计
                LOGI("货满了或者没时间了，直接去虚拟点：");
                ships[i].info();
                berths[ships[i].berthId].info();
                Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                ships[i].goStatus();
                shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
            }
            // 在虚拟点，直接出发
            else if(ships[i].berthId == -1){
                // 分配指定泊位
                int assignedBerthId = allocationBerth(ships[i].id,ships,berths);
                ships[i].berthId = assignedBerthId;
                LOGI("在虚拟点，直接出发前往泊位：",assignedBerthId);
                ships[i].info();
                shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),assignedBerthId}));
            }
            else{
                // 到了终局准备阶段前直接走;一去一回
                // if(berths[ships[i].berthId].mustGo(remainder,2 * berths[ships[i].berthId].time + maxLoadTime) && nowCapaityProportion < 0.5){
                if(berths[ships[i].berthId].mustGo(remainder,2 * maxTime + maxLoadTime) && nowCapaityProportion < 0.9){
                    //装满,去往虚拟点
                    LOGI("进入终局准备阶段，直接去虚拟点：");
                    ships[i].info();
                    berths[ships[i].berthId].info();
                    Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                    ships[i].goStatus();
                    shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                }
                // 在预定泊位上并且还有时间去其他泊位装货
                else if(inAssignedBerth(ships[i].berthId) && berths[ships[i].berthId].mustGo(remainder,500 * 2 + maxLoadTime * 2)){
                    // 找到当前 不是预定泊位 && 货物量挺多 && 没有船搬运的泊位
                    int berthId = findResidueBerth(berths,ships);
                    if(berthId != -1 && berthId != ships[i].berthId){
                        LOGI("还有时间，去其他泊位装货：");
                        ships[i].info();
                        berths[ships[i].berthId].info();
                        berths[berthId].info();
                        // 恢复选定泊位
                        // berth2Ship[ships[i].berthId] = -1;
                        // ship2Berth[ships[i].id] = berthId;
                        ships[i].berthId = berthId;
                        shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),berthId}));
                        
                    }
                }
                //  货物数量为0
                else if(berths[ships[i].berthId].reached_goods.size() == 0){
                    // 泊位不一致
                    if(!inAssignedBerth(ships[i].berthId)){
                        // 剩余货量 >= 70%，去对应泊位;否则去虚拟点
                        if(nowCapaityProportion < 0.2){
                            Berth::deliverGoodNum += (ships[i].capacity - ships[i].now_capacity);
                            LOGI("不在预定泊位上，当前剩余容量:",nowCapaityProportion,",去虚拟点");
                            ships[i].info();
                            ships[i].goStatus();
                            shipActions.push_back(std::make_pair(i, Action{DEPART_BERTH,Point2d(),-1}));
                        }
                        else{
                            int assignedId = allocationBerth(ships[i].id,ships,berths);
                            LOGI("不在预定泊位上，当前剩余容量:",nowCapaityProportion,",去预定泊位");
                            ships[i].info();
                            ships[i].berthId = assignedId;
                            shipActions.push_back(std::make_pair(ships[i].id, Action{MOVE_TO_BERTH,Point2d(),assignedId}));
                        }
                        berths[ships[i].berthId].info();
                    }
                    
                }
                // 有货拿货
                else{
                    int shipment = std::min(static_cast<int>(berths[ships[i].berthId].reached_goods.size()),berths[ships[i].berthId].velocity);
                    int res = ships[i].loadGoods(shipment); // 装货
                    berths[ships[i].berthId].reached_goods.erase(berths[ships[i].berthId].reached_goods.begin(),berths[ships[i].berthId].reached_goods.begin() + res);
                }
            }
            break;
        case 0:
            break;
        }
    }
    LOGI("成功送达比例：--------------------------------");
    berths[0].info();
    if(debug && currentFrame> 14990&& currentFrame < 14995){
        for (std::unordered_map<int, int>::iterator it = berth2Ship.begin(); it != berth2Ship.end(); ++it) {
            LOGI("分配的五个泊位：",it->first,",",it->second);
        }
    }
    return shipActions;
}
