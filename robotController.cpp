#include "robotController.h"

void RobotController::runController(const Map &map)
{
    // 为所有需要寻路算法的机器人调用寻路算法，给定新目标位置
    for (Robot &robot : robots)
        if (needPathfinding(robot))
            runPathfinding(map, robot);

    // 更新所有机器人下一步位置
    for (Robot &robot : robots)
        robot.updateNextPos();

    while(1){
        // 考虑下一步机器人的行动是否会冲突
        std::set<RobotController::CollisionEvent> collisions = detectNextFrameConflict();
        if(collisions.empty())
            break;

        // 遍历冲突机器人集合
        for(const auto &collision : collisions) {
            // 重新规划冲突机器人的行动以解决冲突
            resolveConflict(collision);
        }
        // 直至解决冲突
    }

    // 返回给 gameManager 以输出所有机器人的行动指令
}

std::set<RobotController::CollisionEvent> RobotController::detectNextFrameConflict()
{
    std::set<RobotController::CollisionEvent> collision; // 使用 set 保证输出的机器人对不重复
    for(const Robot &robot1 : robots){
        for(const Robot &robot2 : robots){
            if(robot1.id == robot2.id)
                continue;

            // 检查下一帧前往位置是否相同
            if(robot1.nextPos == robot2.nextPos){
                CollisionEvent event(robot1.id, robot2.id, CollisionEvent::TargetOverlap);
                collision.insert(event);
            }
            // 检查是否互相前往对方当前所在地
            else if(robot1.nextPos == robot2.pos && robot1.pos == robot2.nextPos){
                CollisionEvent event(robot1.id, robot2.id, CollisionEvent::SwapPositions);
                collision.insert(event);
            }
        }
    }
    return collision;
}

void RobotController::resolveConflict(const CollisionEvent &event)
{
    Robot &robot1 = robots[event.robotId1];
    Robot &robot2 = robots[event.robotId2];
    // DIZZY 状态的 robot 不可以移动
    // 重新寻路或等待，根据它们的代价来判断，或者往空位走一格

    // 下一帧前往位置相同
    if(event.type == CollisionEvent::CollisionType::TargetOverlap){
        // 下一帧是 robot1 和 robot2 的 destination
        // 下一帧是 robot1 的 destination
        // 下一帧是 robot1 的 destination
        // 下一帧都不是它们的 destination，都只是过路，停一帧
        // 无法解决？
    }
    // 下一帧分别前往它们当前帧的位置
    else if(event.type == CollisionEvent::CollisionType::SwapPositions){
        // robot2 当前帧位置是 robot1 的 destination, robot2 只是过路。robot1 等待，robot2 重新寻路
        // robot1 当前帧位置是 robot2 的 destination，robot1 只是过路。robot2 等待，robot1 重新寻路
        // robot1 和 robot2 都是过路，选择一个重新寻路
        // robot1 和 robot2 处在单形道上
        // 无法解决？
    }
}

bool RobotController::resolveDeadlocks()
{
    // 解决死锁的逻辑
    // 根据机器人的位置和预定路径检测潜在的死锁
    // 如果检测到死锁，尝试通过调整任务分配、路径或优先级来解决
    return true; // 返回值表示是否成功解决了死锁
}

void RobotController::runPathfinding(const Map &map, Robot &robot)
{
    // 寻路不成功，设置机器人状态
    if (!robot.findPath(map)){
        robot.path = Path();
        robot.targetid = -1;
        robot.destination = Point2d(-1,-1);
    }
    // 寻路成功，设置机器人状态
    else{
        ;
    }
}


bool RobotController::needPathfinding(const Robot &robot)
{
    if (robot.status==MOVING_TO_GOODS && robot.targetid!=-1 && robot.path.empty()) {
        return true;
    }
    if (robot.status==MOVING_TO_BERTH && robot.targetid!=-1 && robot.path.empty()) {
        return true;
    }
    return false;
}