#pragma once
#include <set>
#include "robot.h"
#include "utils.h"
#include "singleLaneManager.h"
#include "log.h"

class RobotController
{
public:
    struct CollisionEvent 
    {
        int robotId1; // 第一个机器人的ID
        int robotId2; // 第二个机器人的ID，如果事件只涉及一个机器人，则此ID可以设置为-1
        enum CollisionType { 
            TargetOverlap,  // 目标重叠
            SwapPositions,  // 交换位置
            HeadOnAttempt,  // 两个机器人从对立方向尝试进入单行路
            EntryAttemptWhileOccupied // 单行道已被占据，另一机器人尝试进入导致冲突
        } type; // 碰撞类型
        
        CollisionEvent(int id1, CollisionType t)
            : robotId1(id1), robotId2(-1), type(t) {}
        CollisionEvent(int id1, int id2, CollisionType t)
            : robotId1(std::min(id1, id2)), robotId2(std::max(id1, id2)), type(t) {}
        bool operator<(const CollisionEvent &rhs) const {
            // 防止重复机器人对
            return robotId1 < rhs.robotId1 || (robotId1 == rhs.robotId1 && robotId2 < rhs.robotId2);
        }
    };
public:
    RobotController(std::vector<Robot> &robots) : robots(robots) {
        refindPathFlag = std::vector<bool>(ROBOTNUMS, false);
        waitFlag = std::vector<bool>(ROBOTNUMS, false);
    }
    // void clearAction()
    // {
    //     robotAction.clear();
    // }

    // void updateRobotAction(int robotId, const Action &action)
    // {
    //     if (robotId < 0 || robotId >= ROBOTNUMS)
    //     {
    //         LOGE("updateRobotAction wrong robotID: ", robotId);
    //         return;
    //     }
    //     robotAction[robotId] = action;
    // }

    // 为所有需要寻路算法的机器人调用寻路算法
    // 更新所有机器人下一步位置
    // while
    // 考虑下一步机器人的行动是否会冲突
    // 解决冲突（重新寻路或等待，根据它们的代价来判断）
    // 直至解决冲突
    // 确定下一步所有机器人的行动
    void runController(Map &map, const SingleLaneManager &singleLaneManager);

private:
    void reset(){
        std::fill(refindPathFlag.begin(), refindPathFlag.end(), false);
        std::fill(waitFlag.begin(), waitFlag.end(), false);
    }
    // 根据动作判断是否需要调用寻路算法
    bool needPathfinding(const Robot &robot);

    // 调用寻路算法
    void runPathfinding(const Map &map, Robot &robot);

    // 检测机器人之间是否冲突，输出冲突的机器人 ID (对)，不考虑地图障碍物的情况
    std::set<CollisionEvent> detectNextFrameConflict(const Map &map, const SingleLaneManager &singleLaneManager);

    // 尝试为所有机器人分配新状态解决冲突
    void tryResolveConflict(Map &map, const CollisionEvent &event);

    // 解决死锁的逻辑，尝试让一个机器人移动往一个可行的点以让出终点
    void resolveDeadlocks(Map &map, Robot &robot1, Robot &robot2);

    // 根据机器人优先级判断应该等待的机器人的引用，优先级低的应该等待
    const Robot & decideWhoWaits(const Robot &robot1, const Robot &robot2);
    // 设置标志位，让一个机器人等待
    void makeRobotWait(const Robot &robot);
    // 设置标志位，让一个机器人重新寻路
    void makeRobotRefindPath(const Robot &robot);
    // 让一个机器人等待
    void stopRobot(Robot &robot);

    void decideWhoToWaitAndRefindWhenTargetOverlap(Map &map, const Robot &robot1, const Robot &robot2);

    void checkRobotsEnteringSingleLanes(Map &map);

private:
    // std::unordered_map<int, Action> robotAction;
    std::vector<Robot> &robots;
    std::vector<bool> refindPathFlag;
    std::vector<bool> waitFlag;
    


};