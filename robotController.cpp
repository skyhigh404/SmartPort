#include "robotController.h"
#include <chrono>
void RobotController::runController(Map &map, const SingleLaneManager &singleLaneManager)
{
    auto start = std::chrono::steady_clock::now();
    // 为所有需要寻路算法的机器人调用寻路算法，给定新目标位置
    for (Robot &robot : robots){
        if (robot.status==DEATH) continue;
        if (needPathfinding(robot)){
            // LOGI("機器人",robot.id,"需要尋路");
            runPathfinding(map, robot);
            // LOGI(robot);
        }
    }
    auto end = std::chrono::steady_clock::now();
    LOGI("robotController 尋路时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");

    // 更新所有机器人下一步位置
    for (Robot &robot : robots)
        robot.updateNextPos();

    start = std::chrono::steady_clock::now();
    int tryTime = 0;
    // 尝试次数大于 0 就出错
    for(; tryTime <= 1; ++tryTime){
        reset();
        updateTemporaryObstacles(map);
        // 考虑下一步机器人的行动是否会冲突
        std::set<CollisionEvent, CollisionEventCompare> collisions = detectNextFrameConflict(map, singleLaneManager);
        if(collisions.empty())
            break;
        LOGI("发现冲突");
        for(const auto &collision : collisions) {
            LOGI(collision.robotId1, ", ", collision.robotId2, ", ", collision.type);
        }
        // 遍历冲突机器人集合
        for(const auto &collision : collisions) {
            // 重新规划冲突机器人的行动以解决冲突
            tryResolveConflict(map, collision);
        }

        // 为需要重新寻路的机器人重新寻路，并设置新的下一帧位置
        for (int i = 0; i < refindPathFlag.size(); ++i) {
            if (refindPathFlag[i]){
                LOGI("重新寻路", robots.at(i));
                runPathfinding(map, robots.at(i));
                robots.at(i).updateNextPos();
            }
        }
        // 为停止一帧的机器人设置下一帧为当前帧位置
        for (int i = 0; i < waitFlag.size(); ++i) {
            if (waitFlag[i]){
                LOGI("等待", robots.at(i));
                stopRobot(robots.at(i));
            }
        }
        // 直至解决冲突
    }

    // for(const auto &robot : robots)
    //     LOGI(robot);
    end = std::chrono::steady_clock::now();
    LOGI("robotController 冲突处理时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms, i: ", tryTime);

    // 返回给 gameManager 以输出所有机器人的行动指令
}

std::set<RobotController::CollisionEvent, RobotController::CollisionEventCompare>
RobotController::detectNextFrameConflict(const Map &map, const SingleLaneManager &singleLaneManager)
{
    std::set<CollisionEvent, CollisionEventCompare> collision; // 使用 set 保证输出的机器人对不重复
    for(size_t i = 0; i < robots.size(); ++i){
        const Robot& robot1 = robots[i];
        for(size_t j = i + 1; j < robots.size(); ++j){
            const Robot& robot2 = robots[j];

            int currentSingleLaneID1 = singleLaneManager.getSingleLaneId(robot1.pos);
            int nextFrameSingleLaneID1 = singleLaneManager.getSingleLaneId(robot1.nextPos);
            int currentSingleLaneID2 = singleLaneManager.getSingleLaneId(robot2.pos);
            int nextFrameSingleLaneID2 = singleLaneManager.getSingleLaneId(robot2.nextPos);
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
            // 检查机器人下一帧是否尝试同时相向进入单行道，同时从同一位置进入单行道已经被 TargetOverlap 排除
            else if(nextFrameSingleLaneID1 >=1 &&
                    currentSingleLaneID1 == 0 &&
                    currentSingleLaneID2 == 0 &&
                    nextFrameSingleLaneID1 == nextFrameSingleLaneID2 &&
                    singleLaneManager.isEnteringSingleLane(nextFrameSingleLaneID1, robot1.nextPos) &&
                    singleLaneManager.isEnteringSingleLane(nextFrameSingleLaneID2, robot2.nextPos)){
                CollisionEvent event(robot1.id, robot2.id, CollisionEvent::HeadOnAttempt);
                collision.insert(event);
            }
            // 检查机器人下一帧是否尝试进入加锁的单行道
            else if(nextFrameSingleLaneID1 >=1 && 
                    currentSingleLaneID1 == 0 && 
                    singleLaneManager.isLocked(nextFrameSingleLaneID1, robot1.nextPos)){
                // const SingleLaneLock& lock = singleLaneManager.getLock(nextFrameSingleLaneID);
                CollisionEvent event(robot1.id, CollisionEvent::EntryAttemptWhileOccupied);
                collision.insert(event);
            }
        }
    }
    return collision;
}

void RobotController::tryResolveConflict(Map &map, const CollisionEvent &event)
{
    // DIZZY 状态的 robot 不可以移动 robot1.status == RobotStatus::DIZZY
    // 重新寻路或等待，根据它们的代价来判断，或者往空位走一格
    Robot &robot1 = robots[event.robotId1];
    Robot &robot2 = (event.robotId2 != -1) ? robots[event.robotId2] : robot1; // 用于单一机器人事件处理
    // 下一帧前往位置相同
    if(event.type == CollisionEvent::CollisionType::TargetOverlap){
        // 检查是否有机器人是静止状态，这代表会直接撞过去
        if (robot1.nextPos == robot1.pos || robot2.nextPos == robot2.pos){
            // robot1 静止, robot1 占据了 robot2 的终点, robot2 停止
            if (robot1.nextPos == robot1.pos && robot1.nextPos == robot2.destination){
                LOGI("robot1.nextPos == robot1.pos && robot1.nextPos == robot2.destination ",robot1," ",robot2);
                makeRobotWait(robot2);
            }
            // robot2 静止, robot2 占据了 robot1 的终点, robot1 停止
            else if (robot2.nextPos == robot2.pos && robot2.nextPos == robot1.destination){
                LOGI("robot2.nextPos == robot2.pos && robot2.nextPos == robot1.destination ",robot1," ",robot2);
                makeRobotWait(robot1);
            }
            // robot1 静止，robot2 重新寻路 // TODO: 不一定非要重新寻路，也可以往旁边让一格
            else if (robot1.nextPos == robot1.pos){
                LOGI("robot1.nextPos == robot1.pos ",robot1," ",robot2);
                // map.addTemporaryObstacle(robot1.pos);
                makeRobotRefindPath(robot2);
            }
            // robot2 静止，robot1 重新寻路
            else if (robot2.nextPos == robot2.pos){
                LOGI("robot2.nextPos == robot2.pos ",robot1," ",robot2);
                // map.addTemporaryObstacle(robot2.pos);
                makeRobotRefindPath(robot1);
            }
        }
        // 检查机器人是否处于DIZZY状态
        else if (robot1.status == RobotStatus::DIZZY || robot2.status == RobotStatus::DIZZY) {
            // 如果任一机器人处于DIZZY状态，则另一机器人重新寻路
            if (robot1.status != RobotStatus::DIZZY) {
                LOGI("DIZZY ",robot2, " 重新寻路", robot1);
                // map.addTemporaryObstacle(robot2.pos);
                makeRobotRefindPath(robot1);
            }
            else if (robot2.status != RobotStatus::DIZZY) {
                LOGI("DIZZY ",robot1, " 重新寻路", robot2);
                // map.addTemporaryObstacle(robot1.pos);
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
            if (robot1.nextPos != robot2.destination && robot2.nextPos != robot1.destination ) {
                // 基于某种逻辑决定谁等待，如果处于单行路，需要额外解决方案
                // 让一个机器人等待，一个机器人重新寻路，有可能它的终点暂时不可达导致寻路失败
                LOGI("robot1.nextPos != robot2.destination && robot2.nextPos != robot1.destination ",robot1," ",robot2);
                // 用解决死锁的逻辑，让一个机器人往旁边移动一个。TODO: 不完善的方案
                decideWhoToWaitAndRefindWhenTargetOverlap(map, robot1, robot2);
                
            }
            // 下一帧是 robot1 和 robot2 的 destination，让一个机器人等待
            else if (robot1.nextPos == robot2.destination && robot2.nextPos == robot1.destination) {
                LOGI("robot1.nextPos == robot2.destination && robot2.nextPos == robot1.destination ",robot1," ",robot2);
                makeRobotWait(decideWhoWaits(robot1, robot2)); // 基于某种逻辑决定谁等待
            }
            // 下一帧是 robot1 的 destination，让 robot2 等待，robot 1 继续向前   //或重新寻路
            else if (robot2.nextPos == robot1.destination) {
                LOGI("robot2.nextPos == robot1.destination ",robot1," ",robot2);
                makeRobotWait(robot2);
            } 
            // 下一帧是 robot2 的 destination，让 robot1 等待，robot 2 继续向前   //或重新寻路
            else if (robot1.nextPos == robot2.destination) {
                LOGI("robot1.nextPos == robot2.destination ",robot1," ",robot2);
                makeRobotWait(robot1);
            }
            // robot1 和 robot2 处在单行道上要怎么解决
            // 未考虑到的情况
            else{
                LOGE("未考虑到的 TargetOverlap, robot id: ", robot1, ", ", robot2);
                // robot1.nextPos == robot2.destination，但是 robot1.path.empty() 为 true, 反之亦然
                makeRobotWait(robot1);
                makeRobotWait(robot2);
            }
        }
    }
    // 下一帧分别前往它们当前帧的位置
    else if(event.type == CollisionEvent::CollisionType::SwapPositions){
        // DIZZY 状态的 robot 不可以移动，因此不应该出现这种状态
        if(robot1.status == RobotStatus::DIZZY || robot2.status == RobotStatus::DIZZY) {
            LOGI("SwapPositions 错误情况出现了 DIZZY, robot id: ", robot1, ", ", robot2);
        }
        // robot2 当前帧位置是 robot1 的 destination, robot1 当前帧位置是 robot2 的 destination
        else if (robot1.destination == robot2.pos && robot1.pos == robot2.destination) {
            LOGI("发生死锁");
            resolveDeadlocks(map, robot1, robot2);  // TODO: 未经验证
        }
        // robot2 当前帧位置是 robot1 的 destination, robot2 只是过路。robot1 等待，robot2 重新寻路
        else if (robot1.destination == robot2.pos && !robot2.path.empty()) {
            LOGI("robot1.destination == robot2.pos && !robot2.path.empty(): ", robot1, ", ", robot2);
            makeRobotWait(robot1);
            // map.addTemporaryObstacle(robot1.pos);
            makeRobotRefindPath(robot2);
        }
        // robot1 当前帧位置是 robot2 的 destination，robot1 只是过路。robot2 等待，robot1 重新寻路
        else if (robot1.pos == robot2.destination && !robot1.path.empty()) {
            LOGI("robot1.pos == robot2.destination && !robot1.path.empty(): ", robot1, ", ", robot2);
            makeRobotWait(robot2);
            // map.addTemporaryObstacle(robot2.pos);
            makeRobotRefindPath(robot1);
        }
        // robot1 和 robot2 都是过路，选择一个重新寻路，让 robot1 等待，robot2 重新寻路
        // 在死路的单行道内无法通过重新寻路解决，进入另一种死锁状态（与目标重合的死锁不同）
        else {
            LOGI("robot1 和 robot2 都是过路，选择一个让一格: ", robot1, ", ", robot2);
            resolveDeadlocks(map, robot1, robot2);
            // LOGI("robot1 和 robot2 都是过路，选择一个重新寻路，让 robot1 等待，robot2 重新寻路: ", robot1, ", ", robot2);
            // makeRobotWait(robot1);
            // map.addTemporaryObstacle(robot1.pos);
            // makeRobotRefindPath(robot2);
        }
        // robot1 和 robot2 处在单形道上要怎么解决
        // 是否考虑了所有情况
    }
    // 机器人下一帧尝试同时相向进入单行道，让优先级低的等待 TODO: 什么情况重新寻路更优
    else if(event.type == CollisionEvent::CollisionType::HeadOnAttempt){
        makeRobotWait(decideWhoWaits(robot1, robot2));
        LOGI("robot1 和 robot2 HeadOnAttempt: ", robot1, ", ", robot2);
    }
    // 机器人下一帧尝试进入加锁的单行道，event只包含一个机器人ID
    else if(event.type == CollisionEvent::CollisionType::EntryAttemptWhileOccupied){
        makeRobotWait(robot1);
        LOGI("robot EntryAttemptWhileOccupied: ", robot1);
    }
}

const Robot & RobotController::decideWhoWaits(const Robot &robot1, const Robot &robot2)
{
    if(robot1.comparePriority(robot2))
        return robot2;
    return robot1;
}

void RobotController::decideWhoToWaitAndRefindWhenTargetOverlap(Map &map, Robot &robot1, Robot &robot2)
{
        // 首先获取两个机器人周围可移动的位置
    const Path robot1Neighbors = map.neighbors(robot1.pos);
    const Path robot2Neighbors = map.neighbors(robot2.pos);
    LOGI("robo1 旁边空位: ", robot1Neighbors.size(), robot1);
    LOGI("robo2 旁边空位: ", robot2Neighbors.size(), robot2);

    // 选择拥有更多移动空间的机器人作为第一移动者
    Robot* firstMover = robot1Neighbors.size() > robot2Neighbors.size() ? &robot1 : &robot2;
    Robot* secondMover = firstMover == &robot1 ? &robot2 : &robot1;

    const Path& firstMoverNeighbors = firstMover == &robot1 ? robot1Neighbors : robot2Neighbors;
    const Path& secondMoverNeighbors = firstMover == &robot1 ? robot2Neighbors : robot1Neighbors;

    // 让可移动空间大的机器人往可移动位置移动一格
    // 尝试移动第一移动者
    for (const Point2d &pos : firstMoverNeighbors) {
        if (pos != secondMover->nextPos) {
            firstMover->moveToTemporaryPosition(pos);
            LOGI("临时移动, 移动空间: ", firstMoverNeighbors.size(), ", 移动位置: ", pos, " ", *firstMover);
            return; // 成功移动后立即返回
        }
    }

    // 如果第一移动者不能移动，尝试移动第二移动者
    for (const Point2d &pos : secondMoverNeighbors) {
        if (pos != firstMover->nextPos) {
            LOGI("临时移动, 移动空间: ", firstMoverNeighbors.size(), ", 移动位置: ", pos, " ", *secondMover);
            secondMover->moveToTemporaryPosition(pos);
            return; // 成功移动后立即返回
        }
    }

    // 如果两个机器人都不能移动
    makeRobotWait(robot1);
    makeRobotWait(robot2);
    LOGE("解决死锁失败");

    // 有可能有某个机器人占了它的终点，只能等待
    // bool robot1DstReachable = robot1.destination != robot2.pos && map.passable(robot1.destination);
    // bool robot2DstReachable = robot2.destination != robot1.pos && map.passable(robot2.destination);
    // if(!robot1DstReachable && !robot2DstReachable){
    //     makeRobotWait(robot1);
    //     makeRobotWait(robot2);
    // }
    // else if(!robot1DstReachable && robot2DstReachable){
    //     makeRobotWait(robot1);
    //     map.addTemporaryObstacle(robot1.pos);
    //     makeRobotRefindPath(robot2);
    // }
    // else if(!robot2DstReachable && robot1DstReachable){
    //     makeRobotWait(robot2);
    //     map.addTemporaryObstacle(robot2.pos);
    //     makeRobotRefindPath(robot1);
    // }
    // else if(robot2DstReachable && robot1DstReachable){
    //     // TODO:临时解决方案，不一定让 robot1 等待
    //     makeRobotWait(robot1);
    //     map.addTemporaryObstacle(robot1.pos);
    //     makeRobotRefindPath(robot2);
    // }
    // else {
    //     LOGE("decideWhoToWaitAndRefindWhenTargetOverlap 未考虑的情况: ", robot1, robot2);
    // }
}

void RobotController::makeRobotWait(const Robot &robot)
{
    auto it = robotResolutionActions.find(robot.id);
    if (it != robotResolutionActions.end()) {
        // 如果找到了，就在对应的向量中添加新的解决方案
        it->second.push_back(ResolutionAction::Wait);
    } else {
        // 如果没有找到，就创建一个新的向量，并添加解决方案
        robotResolutionActions[robot.id] = std::vector<ResolutionAction>{ResolutionAction::Wait};
    }
}
void RobotController::makeRobotRefindPath(const Robot &robot)
{
    auto it = robotResolutionActions.find(robot.id);
    if (it != robotResolutionActions.end()) {
        // 如果找到了，就在对应的向量中添加新的解决方案
        it->second.push_back(ResolutionAction::RefindPath);
    } else {
        // 如果没有找到，就创建一个新的向量，并添加解决方案
        robotResolutionActions[robot.id] = std::vector<ResolutionAction>{ResolutionAction::RefindPath};
    }
}
void RobotController::makeRobotMoveToTempPos(const Robot &robot)
{
    auto it = robotResolutionActions.find(robot.id);
    if (it != robotResolutionActions.end()) {
        // 如果找到了，就在对应的向量中添加新的解决方案
        it->second.push_back(ResolutionAction::MoveAside);
    } else {
        // 如果没有找到，就创建一个新的向量，并添加解决方案
        robotResolutionActions[robot.id] = std::vector<ResolutionAction>{ResolutionAction::MoveAside};
    }
}


void RobotController::resolveDeadlocks(Map &map, Robot &robot1, Robot &robot2)
{
    // 首先获取两个机器人周围可移动的位置
    const Path robot1Neighbors = map.neighbors(robot1.pos);
    const Path robot2Neighbors = map.neighbors(robot2.pos);

    // 选择拥有更多移动空间的机器人作为第一移动者
    Robot* firstMover = robot1Neighbors.size() > robot2Neighbors.size() ? &robot1 : &robot2;
    Robot* secondMover = firstMover == &robot1 ? &robot2 : &robot1;

    const Path& firstMoverNeighbors = firstMover == &robot1 ? robot1Neighbors : robot2Neighbors;
    const Path& secondMoverNeighbors = firstMover == &robot1 ? robot2Neighbors : robot1Neighbors;

    // 让可移动空间大的机器人往可移动位置移动一格
    // 尝试移动第一移动者
    for (const Point2d &pos : firstMoverNeighbors) {
        if (pos != secondMover->pos) {
            firstMover->moveToTemporaryPosition(pos);
            LOGI("临时移动, 移动空间: ", firstMoverNeighbors.size(), ", 移动位置: ", pos, " ", *firstMover);
            return; // 成功移动后立即返回
        }
    }

    // 如果第一移动者不能移动，尝试移动第二移动者
    for (const Point2d &pos : secondMoverNeighbors) {
        if (pos != firstMover->pos) {
            LOGI("临时移动, 移动空间: ", firstMoverNeighbors.size(), ", 移动位置: ", pos, " ", *secondMover);
            secondMover->moveToTemporaryPosition(pos);
            return; // 成功移动后立即返回
        }
    }

    // 如果两个机器人都不能移动
    makeRobotWait(robot1);
    makeRobotWait(robot2);
    LOGE("解决死锁失败");
}

void RobotController::runPathfinding(const Map &map, Robot &robot)
{
    // 寻路不成功，设置机器人状态
    if (!robot.findPath(map)){
        robot.path = Path();
        robot.targetid = -1;
        robot.destination = Point2d(-1,-1);
        LOGI("尋路失敗",robot);
    }
    // 寻路成功，设置机器人状态
    else{
        LOGI("寻路成功",robot);
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

void RobotController::updateTemporaryObstacles(Map &map)
{
    for (const Robot& robot : robots) {
        map.addTemporaryObstacle(robot.nextPos);
    }
}
