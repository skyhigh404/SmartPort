#include "robotController.h"

void RobotController::runController(Map &map)
{
    // 为所有需要寻路算法的机器人调用寻路算法，给定新目标位置
    for (Robot &robot : robots)
        if (needPathfinding(robot))
            runPathfinding(map, robot);

    // 更新所有机器人下一步位置
    for (Robot &robot : robots)
        robot.updateNextPos();

    while(1){
        reset();
        // 考虑下一步机器人的行动是否会冲突
        std::set<RobotController::CollisionEvent> collisions = detectNextFrameConflict();
        if(collisions.empty())
            break;

        // 遍历冲突机器人集合
        for(const auto &collision : collisions) {
            // 重新规划冲突机器人的行动以解决冲突
            tryResolveConflict(map, collision);
        }

        // 为需要重新寻路的机器人重新寻路，并设置新的下一帧位置
        for (int i = 0; i < refindPathFlag.size(); ++i) {
            if (refindPathFlag[i]){
                runPathfinding(map, robots.at(i));
                robots.at(i).updateNextPos();
            }
        }
        // 为停止一帧的机器人设置下一帧为当前帧位置
        for (int i = 0; i < waitFlag.size(); ++i) {
            if (waitFlag[i])
                stopRobot(robots.at(i));
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

            // 检查下一帧前往位置是否相同，移动机器人撞上静止机器人也在这种情况内
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

void RobotController::tryResolveConflict(Map &map, const CollisionEvent &event)
{
    Robot &robot1 = robots[event.robotId1];
    Robot &robot2 = robots[event.robotId2];
    
    // DIZZY 状态的 robot 不可以移动 robot1.status == RobotStatus::DIZZY
    // 重新寻路或等待，根据它们的代价来判断，或者往空位走一格

    // 下一帧前往位置相同
    if(event.type == CollisionEvent::CollisionType::TargetOverlap){
        // 检查机器人是否处于DIZZY状态
        if (robot1.status == RobotStatus::DIZZY || robot2.status == RobotStatus::DIZZY) {
            // 如果任一机器人处于DIZZY状态，则另一机器人重新寻路
            if (robot1.status != RobotStatus::DIZZY) {
                map.addTemporaryObstacle(robot2.pos);
                makeRobotRefindPath(robot1);
            }
            else if (robot2.status != RobotStatus::DIZZY) {
                map.addTemporaryObstacle(robot1.pos);
                makeRobotRefindPath(robot2);
            }
            // 如果两个都处于 DIZZY 那么就不应该检测到冲突
            else{
                LOGE("两个都处于 DIZZY 不应该检测到冲突 Robot id ", robot1.id, ", ", robot2.id);
            }
        }
        // 两个机器人都可正常运行
        else{
            // 如果下一帧都不是两者的目的地，都只是过路，则停一帧或重新寻路
            if (robot1.nextPos != robot1.destination && robot2.nextPos != robot2.destination) {
                // makeRobotWait(decideWhoWaits(robot1, robot2)); // 基于某种逻辑决定谁等待，如果处于单行路，需要额外解决方案
                // TODO：临时方案，让一个机器人等待，一个机器人重新寻路
                makeRobotWait(robot1);
                map.addTemporaryObstacle(robot1.pos);
                makeRobotRefindPath(robot2);
            }
            // 下一帧是 robot1 和 robot2 的 destination，让一个机器人等待
            else if (robot1.nextPos == robot1.destination && robot2.nextPos == robot2.destination) {
                makeRobotWait(decideWhoWaits(robot1, robot2)); // 基于某种逻辑决定谁等待
            }
            // 下一帧是 robot1 的 destination，让 robot2 等待   //或重新寻路
            else if (robot1.nextPos == robot1.destination) {
                makeRobotWait(robot2);
            } 
            // 下一帧是 robot2 的 destination，让 robot1 等待   //或重新寻路
            else if (robot2.nextPos == robot2.destination) {
                makeRobotWait(robot1);
            }
            // robot1 和 robot2 处在单形道上要怎么解决
            // 未考虑到的情况
            else{
                LOGE("未考虑到的 TargetOverlap, robot id: ", robot1.id, ", ", robot2.id);
            }
        }
    }
    // 下一帧分别前往它们当前帧的位置
    else if(event.type == CollisionEvent::CollisionType::SwapPositions){
        // DIZZY 状态的 robot 不可以移动，因此不应该出现这种状态
        if(robot1.status == RobotStatus::DIZZY || robot2.status == RobotStatus::DIZZY) {
            LOGE("SwapPositions 错误情况出现了 DIZZY, robot id: ", robot1.id, ", ", robot2.id);
        }
        // robot2 当前帧位置是 robot1 的 destination, robot1 当前帧位置是 robot2 的 destination
        else if (robot1.destination == robot2.pos && robot1.pos == robot2.destination) {
            LOGE("发生死锁");
            // resolveDeadlocks();  // TODO
        }
        // robot2 当前帧位置是 robot1 的 destination, robot2 只是过路。robot1 等待，robot2 重新寻路
        else if (robot1.destination == robot2.pos && !robot2.path.empty()) {
            makeRobotWait(robot1);
            map.addTemporaryObstacle(robot1.pos);
            makeRobotRefindPath(robot2);
        }
        // robot1 当前帧位置是 robot2 的 destination，robot1 只是过路。robot2 等待，robot1 重新寻路
        else if (robot1.pos == robot2.destination && !robot1.path.empty()) {
            makeRobotWait(robot2);
            map.addTemporaryObstacle(robot2.pos);
            makeRobotRefindPath(robot1);
        }
        // robot1 和 robot2 都是过路，选择一个重新寻路，让 robot1 等待，robot2 重新寻路
        else {
            makeRobotWait(robot1);
            map.addTemporaryObstacle(robot1.pos);
            makeRobotRefindPath(robot2);
        }
        // robot1 和 robot2 处在单形道上要怎么解决
        // 是否考虑了所有情况
    }
}

const Robot & RobotController::decideWhoWaits(const Robot &robot1, const Robot &robot2)
{
    if(robot1.comparePriority(robot2))
        return robot1;
    return robot2;
}
void RobotController::makeRobotWait(const Robot &robot)
{
    waitFlag[robot.id] = true;
}
void RobotController::makeRobotRefindPath(const Robot &robot)
{
    refindPathFlag[robot.id] = true;
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

void RobotController::stopRobot(Robot &robot)
{
    robot.nextPos = robot.pos;
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