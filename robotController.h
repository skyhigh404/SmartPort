#pragma once
#include <set>
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
        if (robotId < 0 || robotId >= ROBOTNUMS)
        {
            LOGE("updateRobotAction wrong robotID: ", robotId);
            return;
        }
        robotAction[robotId] = action;
    }

    // 为所有需要寻路算法的机器人调用寻路算法
    // 更新所有机器人下一步位置
    // while
    // 考虑下一步机器人的行动是否会冲突
    // 解决冲突（重新寻路或等待，根据它们的代价来判断）
    // 直至解决冲突
    // 确定下一步所有机器人的行动
    void runController(const Map &map);

private:
    // 根据动作判断是否需要调用寻路算法
    bool needPathfinding(const Action &action);

    // 调用寻路算法
    void runPathfinding(const Map &map, Robot &robot);

    // 检测机器人之间是否冲突，输出冲突的机器人 ID 对
    std::set<std::pair<int, int>> detectNextFrameConflict();

    // 解决冲突
    void resolveConflict(Robot &robot1, Robot &robot2);

    // 解决死锁的逻辑
    // 根据机器人的位置和预定路径检测潜在的死锁
    // 如果检测到死锁，尝试通过调整任务分配、路径或优先级来解决
    // 返回值表示是否成功解决了死锁
    bool resolveDeadlocks();



private:
    std::unordered_map<int, Action> robotAction;
    std::vector<Robot> &robots;
    
    struct CollisionEvent 
    {
        int robotId1; // 第一个机器人的ID
        int robotId2; // 第二个机器人的ID
        enum CollisionType { TargetOverlap, SwapPositions } type; // 碰撞类型
        Point2d position; // 发生碰撞的位置（如果是TargetOverlap，这是重叠的目标位置；如果是SwapPositions，这是机器人之一的当前位置）
        
        CollisionEvent(int id1, int id2, CollisionType t, Point2d pos)
            : robotId1(std::min(id1, id2)), robotId2(std::max(id1, id2)), type(t), position(pos) {}
        bool operator<(const CollisionEvent &rhs) const {
            // 防止重复机器人对
            return robotId1 < rhs.robotId1 || (robotId1 == rhs.robotId1 && robotId2 < rhs.robotId2);
        }
    };

};