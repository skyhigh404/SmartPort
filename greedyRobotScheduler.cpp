#include "greedyRobotScheduler.h"

std::vector<std::pair<RobotID, RobotActionSpace::RobotAction>>
GreedyRobotScheduler::scheduleRobots(const Map &map,
                                     const std::vector<Robot> &robots,
                                     std::vector<Goods> &goods,
                                     const std::vector<Berth> &berths,
                                     const int currentFrame)
{
    using namespace RobotActionSpace;

    countRobotsPerBerth(robots);

    std::vector<std::pair<RobotID, RobotAction>> robotActions;
    for (const Robot &robot : robots)
    {
        // 机器人需要寻找合适的货物
        // TODO: 机器人临时改变之前拿取货物的决策，去拿取另一个货物
        if (shouldFetchGoods(robot))
        {
            robotActions.emplace_back(robot.id, findGoodsForRobot(map, robot, goods, berths, currentFrame));
        }
        // 机器人需要寻找合适的泊位
        else if (shouldMoveToBerth(robot))
        {
            robotActions.emplace_back(robot.id, findBerthForRobot(robot, berths));
        }
        // 机器人保持之前的决策
        else
        {
            robotActions.emplace_back(robot.id, RobotAction(RobotActionType::CONTINUE));
        }
    }
    // 返回机器人接下来应当采取的行动，在 gameManager 里对机器人状态进行修改
    return robotActions;
}

void GreedyRobotScheduler::setParameter(const Params &params)
{
}

bool GreedyRobotScheduler::shouldFetchGoods(const Robot &robot)
{
}

bool GreedyRobotScheduler::shouldMoveToBerth(const Robot &robot)
{
}

std::pair<RobotID, RobotActionSpace::RobotAction>
GreedyRobotScheduler::findGoodsForRobot(const Map &map,
                                        const Robot &robot,
                                        std::vector<Goods> &goods,
                                        const std::vector<Berth> &berths,
                                        const int currentFrame)
{
    // 筛选出可用泊位
    // getAvailableBerths(robot);
    // 获取可用的货物子集
    // 注：reference_wrapper封装的元素要用 .get() 获取原对象
    // std::vector<std::reference_wrapper<Goods>> availableGoods = getAvailableGoods();

    // 货物到每个泊位的距离已经在 distsToBerths 中存储，提取第一个距离，该功能封装在一个函数里

    // 计算机器人到每个货物的距离，该功能封装在一个函数里

    // 两个距离相加，得到总的搬运距离

    // 输入距离和货物，计算得分，该功能封装在一个函数里

    // 如果预定了某个货物，需要对货物的状态进行修改，将对货物的状态修改**封装为货物的一个函数**然后调用

    // 选择得分第一的作为搬运目标，构造 pair 并返回
}

std::pair<RobotID, RobotActionSpace::RobotAction>
GreedyRobotScheduler::findBerthForRobot(const Robot &robot,
                                        const std::vector<Berth> &berths)
{
    // 查询机器人所持有货物被分配的泊位 ID，直接构造 pair 并返回
    // 不考虑泊位 isEnabled 为 False 的情况，这应该由其他函数更新所有货物被分配的泊位 ID，而不是由该调度函数负责
    // 但是要检查 berths isEnabled的情况，如果为 False， LOGE 记录。
}