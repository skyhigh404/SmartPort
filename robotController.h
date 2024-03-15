#pragma once
#include "robot.h"
#include "utils.h"
#include "log.h"

class RobotController
{
public:
    RobotController(std::vector<Robot> &robots) : robots(robots) {}
    void clearAction()
    {
        robotAction.clear();
    }

    void updateRobotAction(int robotId, const Action &action)
    {
        if(robotId < 0 || robotId >= ROBOTNUMS){
            LOGE("updateRobotAction wrong robotID: ", robotId);
            return;
        }
        robotAction[robotId] = action;
    }

    void runController()
    {
        // 为所有需要寻路算法的机器人调用寻路算法

        // 更新所有机器人下一步位置

        // while
        // 考虑下一步机器人的行动是否会冲突
        // 解决冲突（重新寻路或等待，根据它们的代价来判断）
        // 直至解决冲突
        // 确定下一步所有机器人的行动
    }

    bool resolveDeadlocks()
    {
        // 解决死锁的逻辑
        // 根据机器人的位置和预定路径检测潜在的死锁
        // 如果检测到死锁，尝试通过调整任务分配、路径或优先级来解决
        return true;// 返回值表示是否成功解决了死锁
    }


private:

    std::unordered_map<int, Action> robotAction;
    std::vector<Robot> &robots;
};