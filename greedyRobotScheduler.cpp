#include "greedyRobotScheduler.h"

GreedyRobotScheduler::GreedyRobotScheduler(std::vector<std::vector<Berth>> &_clusters, std::vector<int> &_berthCluster)
    : clusters(_clusters), berthCluster(std::make_shared<std::vector<int>>(_berthCluster))
    // : clusters(_clusters), berthCluster(_berthCluster)
{
    // assignment = vector<int>(10, -1);
}

void GreedyRobotScheduler::scheduleRobots(const Map &map,
                                          std::vector<Robot> &robots,
                                          std::vector<Goods> &goods,
                                          const std::vector<Berth> &berths,
                                          const int currentFrame)
{
    // LOGI("货物数量：", goods.size());
    // countRobotsPerBerth(robots);
    if (assignment.empty() && robots.size()==maxRobotNum) {
        assignRobotsByCluster(robots, map, ASSIGNBOUND);
    }

    for (Robot &robot : robots)
    {
        if (robot.status==DEATH) continue;
        // 机器人需要寻找合适的货物
        // TODO: 机器人临时改变之前拿取货物的决策，去拿取另一个货物
        if (shouldFetchGoods(robot))
        {
            findGoodsForRobot(map, robot, goods, berths, currentFrame);
        }
        // 机器人需要寻找合适的泊位
        else if (shouldMoveToBerth(robot))
        {
            findBerthForRobot(robot, goods, berths, map);
        }
        // 机器人保持之前的决策
        else
        {
        }
    }
    // 返回机器人接下来应当采取的行动，在 gameManager 里对机器人状态进行修改
    return;
}

void GreedyRobotScheduler::setParameter(const Params &params)
{
    TTL_Bound = params.TTL_Bound;
    TTL_ProfitWeight = params.TTL_ProfitWeight;
    PartitionScheduling = params.PartitionScheduling;
    maxRobotNum = params.maxRobotNum;
    ASSIGNBOUND = params.ASSIGNBOUND;
}

void GreedyRobotScheduler::assignRobotsByCluster(vector<Robot> &robots, const Map &map, vector<int> assignBound)
{
    if (assignment.empty()) 
        assignment = vector<int>(robots.size(), -1);
        
    vector<bool> assigned(robots.size(), false);
    if (assignBound.empty())
    {
        assignBound = vector<int>(clusters.size());
        int count = 0;
        for (int i = 0; i < clusters.size(); i++) {
            assignBound[i] = clusters[i].size();
            count += clusters[i].size();
        }
        while(count<robots.size()) {
            for (int i=0;i<clusters.size();i++) 
                if (count<robots.size()) {
                    assignBound[i]++;
                    count++;
                }
        }
    }
    vector<int> assignNum(clusters.size(), 0);
    // 先每个类分配一个机器人
    for (int i = 0; i < clusters.size(); i++)
    {
        if (assignNum[i] >= assignBound[i])
            continue;
        int mini_dist = INT_MAX, argmin = -1;
        for (int j = 0; j < robots.size(); j++)
        {
            if (assigned[j])
                continue;
            Robot &robot = robots[j];
            int dist = INT_MAX;
            for (int k = 0; k < clusters[i].size(); k++)
            {
                if (dist > map.berthDistanceMap.at(clusters[i][k].id)[robot.pos.x][robot.pos.y])
                {
                    dist = map.berthDistanceMap.at(clusters[i][k].id)[robot.pos.x][robot.pos.y];
                }
            }
            if (dist < mini_dist)
            {
                mini_dist = dist;
                argmin = robot.id;
            }
        }
        assigned[argmin] = true;
        assignment[argmin] = i;
        assignNum[i]++;
    }

    // 再根据机器人离类的距离分配机器人到哪个类去
    for (int j = 0; j < robots.size(); j++)
    {
        if (assigned[j])
            continue;
        Robot &robot = robots[j];
        int mini_dist = INT_MAX, argmin = -1;
        for (int i = 0; i < clusters.size(); i++)
        {
            int dist = INT_MAX;
            // 每个类至多分配多少个机器人
            if (assignNum[i] >= assignBound[i])
                continue; 
            for (int k = 0; k < clusters[i].size(); k++)
            {
                if (dist > map.berthDistanceMap.at(clusters[i][k].id)[robot.pos.x][robot.pos.y])
                {
                    dist = map.berthDistanceMap.at(clusters[i][k].id)[robot.pos.x][robot.pos.y];
                }
            }
            if (dist < mini_dist)
            {
                mini_dist = dist;
                argmin = i;
            }
        }
        assigned[robot.id] = true;
        assignment[robot.id] = argmin;
        assignNum[argmin]++;
    }
    return;
}

// void GreedyRobotScheduler::reassignRobotsByCluster(vector<Goods> &goods, vector<Robot> &robots, Map &map, std::vector<Berth> &berths)
// {
//     // 根据类收益分配机器人
//     vector<int> assignBound(clusters.size(), 0);
//     for (int i = 0; i < assignment.size(); i++)
//         assignBound[assignment[i]]++;

//     // 需要统计（机器人空闲率）和泊位类的价值
//     vector<int> clusterValue(clusters.size(), 0);
//     // calCostAndBestBerthIndes(map, goods, berths);
//     for (auto &good : goods)
//     {
//         if (good.status == 0)
//         {
//             clusterValue[berthCluster[bestBerthIndex[good.id][0]]] += good.value / cost2berths[good.id][bestBerthIndex[good.id][0]];
//         }
//     }
//     int clusterValue_avg = std::accumulate(clusterValue.begin(), clusterValue.end(), 0.0) / clusterValue.size();

//     // 将机器人从价值低的类中释放
//     int freeRobotNum = 0;
//     for (int i = 0; i < clusters.size(); i++)
//     {
//         if (freeRobotNum >= 2)
//             break;
//         if (clusterValue[i] < 0.8 * clusterValue_avg && assignBound[i] > 0)
//         {
//             assignBound[i]--;
//             freeRobotNum++;
//         }
//     }

//     // 将自由机器人分配给高价值类
//     while (freeRobotNum > 0)
//     {
//         auto max_iter = std::max_element(clusterValue.begin(), clusterValue.end());
//         int max_index = std::distance(clusterValue.begin(), max_iter);
//         assignBound[max_index]++;
//         freeRobotNum--;
//         clusterValue[max_index] = 0; // 此类不再参与分配
//     }
//     if (freeRobotNum == 0)
//         return;

//     assignRobotsByCluster(robots, map, assignBound);
// }

bool GreedyRobotScheduler::shouldFetchGoods(const Robot &robot)
{
    // 机器人没携带货物，并且没有目标
    if (robot.carryingItem == 0 && robot.targetid == -1)
        return true;
    return false;
}

bool GreedyRobotScheduler::shouldMoveToBerth(const Robot &robot)
{
    // 机器人携带货物，并且没有目标
    if (robot.carryingItem == 1 && robot.carryingItemId != -1 && robot.targetid == -1)
        return true;
    return false;
}

int GreedyRobotScheduler::WhereIsRobot(const Robot &robot, const std::vector<Berth> &berths, const Map &map)
{
    for (const Berth &berth : berths)
    {
        if (map.cost(robot.pos, berth.pos) <= 6)
            return berth.id;
    }
    return -1;
}

std::vector<std::reference_wrapper<Goods>>
GreedyRobotScheduler::getAvailableGoods(std::vector<Goods> &goods)
{
    std::vector<std::reference_wrapper<Goods>> availableGoods;
    for (Goods &good : goods)
    {
        if (good.status == 0)
        {
            availableGoods.push_back(std::ref(good));
        }
    }
    return availableGoods;
}

vector<long long>
GreedyRobotScheduler::Cost_RobotToGood(const Robot &robot,
                                       std::vector<std::reference_wrapper<Goods>> &availableGoods,
                                       const std::vector<Berth> &berths,
                                       const Map &map)
{
    vector<long long> cost_robot2good(availableGoods.size(), 0);
    for (int j = 0; j < availableGoods.size(); j++)
    {
        if (availableGoods[j].get().status != 0)
        {
            cost_robot2good[j] = INT_MAX;
            continue;
        }
        // 机器人到货物的距离
        int berthid = WhereIsRobot(robot, berths, map);
        if (berthid == -1)
        {
            bool canReach = false;
            for (int k = 0; k < berths.size(); k++)
            {
                if (map.berthDistanceMap.at(k)[robot.pos.x][robot.pos.y] != INT_MAX && map.berthDistanceMap.at(k)[availableGoods[j].get().pos.x][availableGoods[j].get().pos.y] != INT_MAX)
                {
                    canReach = true;
                    break;
                }
            }
            if (!canReach)
                cost_robot2good[j] = INT_MAX;
            else
                cost_robot2good[j] = map.cost(robot.pos, availableGoods[j].get().pos);
        }
        else
            cost_robot2good[j] = map.berthDistanceMap.at(berthid)[availableGoods[j].get().pos.x][availableGoods[j].get().pos.y];
    }
    return cost_robot2good;
}

vector<long long>
GreedyRobotScheduler::Cost_GoodToBerth(std::vector<std::reference_wrapper<Goods>> &availableGoods,
                                       const Map &map)
{
    vector<long long> cost_good2berth(availableGoods.size(), 0);
    for (int j = 0; j < availableGoods.size(); j++)
    {
        // 进入终局的时候要更新distsToBerths
        // LOGI(j, " cost_good2berth ", availableGoods[j].get().distsToBerths.empty());
        if (availableGoods[j].get().distsToBerths.empty()) cost_good2berth[j] = INT_MAX;
        else cost_good2berth[j] = availableGoods[j].get().distsToBerths[0].second;
    }
    return cost_good2berth;
}

std::pair<vector<float>, vector<int>>
GreedyRobotScheduler::getProfitsAndSortedIndex(std::vector<std::reference_wrapper<Goods>> &availableGoods,
                                               vector<long long> &cost_robot2good,
                                               vector<long long> &cost_good2berth)
{
    // 计算收益
    vector<float> profits(availableGoods.size(), 0);
    for (int j = 0; j < availableGoods.size(); j++)
    {
        if (cost_robot2good[j] >= INT_MAX || cost_good2berth[j] >= INT_MAX)
            continue;
        profits[j] = availableGoods[j].get().value * 1.0 / (cost_robot2good[j] + cost_good2berth[j]);

        if (availableGoods[j].get().TTL <= TTL_Bound && !enterFinal)
            profits[j] *= TTL_ProfitWeight;
    }
    // 对收益排序
    std::vector<int> indices(availableGoods.size(), 0);
    for (int j = 0; j < availableGoods.size(); j++)
        indices[j] = j;
    std::sort(indices.begin(), indices.end(), [&](int a, int b)
              {
                  return profits[a] > profits[b]; // 根据第二个维度进行降序排序
              });

    return std::make_pair(profits, indices);
}

void GreedyRobotScheduler::findGoodsForRobot(const Map &map,
                                             Robot &robot,
                                             std::vector<Goods> &goods,
                                             const std::vector<Berth> &berths,
                                             const int currentFrame)
{
    // 获取可用的货物子集
    // 注：reference_wrapper封装的元素要用 .get() 获取原对象
    std::vector<std::reference_wrapper<Goods>> availableGoods = getAvailableGoods(goods);

    // 计算机器人到货物的距离
    vector<long long> cost_robot2good = Cost_RobotToGood(robot, availableGoods, berths, map);

    // 计算机器人到每个货物的距离，该功能封装在一个函数里
    vector<long long> cost_good2berth = Cost_GoodToBerth(availableGoods, map);

    // 输入距离和货物，计算得分，该功能封装在一个函数里
    auto profitsAndSortedIndex = getProfitsAndSortedIndex(availableGoods, cost_robot2good, cost_good2berth);
    vector<float> profits = profitsAndSortedIndex.first;
    vector<int> index = profitsAndSortedIndex.second;

    // 选择得分第一的作为搬运目标
    for (int j = 0; j < availableGoods.size(); ++j)
    {
        int goodIndex = index[j];
        Goods &good = availableGoods[goodIndex].get();
        // int berthsIndex = bestBerthIndex[goodsIndex];
        int timeToGoods = cost_robot2good[goodIndex];
        int timeToBerths = cost_good2berth[goodIndex];

        // 集中往一个泊位搬货
        if (assignedBerthID != -1) {
            timeToBerths = map.berthDistanceMap.at(assignedBerthID)[good.pos.x][good.pos.y];
        }

        if (timeToBerths == INT_MAX || timeToGoods == INT_MAX)
            continue;
        int berthsIndex = good.distsToBerths[0].first;
        // LOGI("货物id：",good.id,"货物状态：",good.status,"货物收益：",profits[good.id]);
        if (PartitionScheduling && !assignment.empty() && robot.id<assignment.size() && !enterFinal && berthCluster->at(berthsIndex)!=assignment[robot.id]) continue;

        if (good.status == 0 && profits[goodIndex] > 0 && good.TTL + 10 >= timeToGoods)
        {
            // LOGI("成功分配货物", goods[goodIndex].id, ",给机器人：", robot.id, "机器人状态：", robot.state);
            robot.assignGoodOrBerth(good.id, good.pos);
            good.assignRobot();
            return;
        }
    }
    LOGI("机器人", robot.id, "分配货物失败");
    return;
}

void GreedyRobotScheduler::findBerthForRobot(Robot &robot,
                                             std::vector<Goods> &goods,
                                             const std::vector<Berth> &berths,
                                             const Map &map)
{
    if (assignedBerthID != -1 && map.berthDistanceMap.at(assignedBerthID)[robot.pos.x][robot.pos.y] < INT_MAX) {
        robot.assignGoodOrBerth(assignedBerthID, berths[assignedBerthID].pos);
        return;
    }

    // 查询机器人所持有货物被分配的泊位 ID，直接构造 pair 并返回
    const Berth &berth = berths[goods[robot.carryingItemId].distsToBerths[0].first];
    // 不考虑泊位 isEnabled 为 False 的情况，这应该由其他函数更新所有货物被分配的泊位 ID，而不是由该调度函数负责
    // 但是要检查 berths isEnabled的情况，如果为 False， LOGE 记录。
    if (!berth.isEnable())
    {
        LOGI("findBerthForRobot：机器人", robot.id, "分配到泊位", berth.id, "。但该泊位不可用");
        robot.assignGoodOrBerth();
        return;
    }

    LOGI("findBerthForRobot：机器人", robot.id, "分配到泊位", berth.id);
    robot.assignGoodOrBerth(berth.id, berth.pos);
}