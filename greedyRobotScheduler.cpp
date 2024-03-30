#include "greedyRobotScheduler.h"

GreedyRobotScheduler::GreedyRobotScheduler(const std::unordered_map<BerthID, int> &cluster)
{
    berthCluster = cluster;
}

void GreedyRobotScheduler::scheduleRobots(const Map &map,
                                          std::vector<Robot> &robots,
                                          std::vector<Goods> &goods,
                                          const std::vector<Berth> &berths,
                                          const int currentFrame)
{
    countRobotsPerBerth(robots);

    for (Robot &robot : robots)
    {
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
}

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
        cost_good2berth[j] = availableGoods[j].get().distsToBerths[0].second;
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
        int berthsIndex = good.distsToBerths[0].first;
        int timeToGoods = cost_robot2good[goodIndex];
        int timeToBerths = cost_good2berth[goodIndex];
        if (timeToBerths == INT_MAX || timeToGoods == INT_MAX)
            continue;
        // LOGI("货物id：",good.id,"货物状态：",good.status,"货物收益：",profits[good.id]);
        // if (PartitionScheduling && !enterFinal && berthCluster[berthsIndex]!=assignment[robot.id]) continue;

        if (good.status == 0 && profits[goodIndex] > 0 && good.TTL + 10 >= timeToGoods)
        {
            LOGI("成功分配货物", goods[goodIndex].id, ",给机器人：", robot.id, "机器人状态：", robot.state);
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
    // 查询机器人所持有货物被分配的泊位 ID，直接构造 pair 并返回
    const Berth &berth = berths[goods[robot.carryingItemId].distsToBerths[0].first];
    // 不考虑泊位 isEnabled 为 False 的情况，这应该由其他函数更新所有货物被分配的泊位 ID，而不是由该调度函数负责
    // 但是要检查 berths isEnabled的情况，如果为 False， LOGE 记录。
    if (!berth.isEnabled)
    {
        LOGI("findBerthForRobot：机器人", robot.id, "分配到泊位", berth.id, "。但该泊位不可用");
        robot.assignGoodOrBerth();
        return;
    }

    Point2d dest(-1, -1);
    int nearest = INT_MAX;
    // 去曼哈顿距离最近且有空的位置
    for (int i = 3; i >= 0; i--)
    {
        for (int j = 3; j >= 0; j--)
        {
            if (map.cost(robot.pos, Point2d(berth.pos.x + i, berth.pos.y + j)) < nearest)
            {
                dest = Point2d(berth.pos.x + i, berth.pos.y + j);
                nearest = map.cost(robot.pos, Point2d(berth.pos.x + i, berth.pos.y + j));
            }
        }
    }
    if (nearest == INT_MAX)
    {
        LOGI("findBerthForRobot：机器人", robot.id, "分配到泊位", berth.id, "失败");
        robot.assignGoodOrBerth();
        return;
    }
    robot.assignGoodOrBerth(berth.id, berth.pos);
}