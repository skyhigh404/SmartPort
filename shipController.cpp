#include "shipController.h"

// 控制船的整体调度
void ShipController::runController(Map &map,std::vector<Ship> &ships, const SingleLaneManager &singleLaneManager){
    // LOGI("shipController::runController");
    // LOGI("ship num:",ships.size());
    auto start = std::chrono::steady_clock::now();
    // 为所有需要寻路算法的船调用寻路算法，给定新目标位置
    for (Ship &ship : ships){
        // LOGI(ship);

        if (ship.state != 0) continue;
        else if (ship.shipStatus != ShipStatusSpace::ShipStatus::MOVING_TO_BERTH &&
        ship.shipStatus != ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY) continue;
        if (needPathfinding(ship)){
            runPathfinding(map, ship);
        }
    }
    auto end = std::chrono::steady_clock::now();
    int countTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if(countTime > 10)
        LOGI("shipController 尋路时间: ",countTime," ms");

    // 更新所有船下一步位置
    for (Ship &ship : ships)
        ship.updateNextPos();

    start = std::chrono::steady_clock::now();
    int tryTime = 0;
    // // 尝试次数大于 0 就出错
    // for(; tryTime <= 2; ++tryTime){
    //     reset();
    //     updateTemporaryObstacles(map);
    //     // 考虑下一步机器人的行动是否会冲突
    //     std::set<CollisionEvent, CollisionEventCompare> collisions = detectNextFrameConflict(map, singleLaneManager);
    //     if(collisions.empty())
    //         break;
    //     LOGI("发现冲突");
    //     for(const auto &collision : collisions) {
    //         LOGI(collision.robotId1, ", ", collision.robotId2, ", ", collision.type);
    //     }
    //     // 遍历冲突机器人集合
    //     for(const auto &collision : collisions) {
    //         // 重新规划冲突机器人的行动以解决冲突
    //         tryResolveConflict(map, collision);
    //     }

    //     // 权衡机器人的规划，设置设定合理的指令
    //     rePlanRobotMove(map);

    //     map.clearTemporaryObstacles();
    //     // 直至解决冲突
    // }

    // for(const auto &robot : robots)
    //     LOGI(robot);
    end = std::chrono::steady_clock::now();
    countTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if(countTime > 10)
        LOGI("shipController 冲突处理时间: ",countTime," ms, i: ", tryTime);
    // LOGI("shipcontroller执行完毕");
}

bool ShipController::needPathfinding(Ship &ship)
{
    if (ship.shipStatus== ShipStatusSpace::ShipStatus::MOVING_TO_BERTH && ship.berthId !=-1 && ship.isDestinationValid() && ship.path.empty()) {
        return true;
    }
    if (ship.shipStatus== ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY && ship.isDestinationValid() && ship.path.empty()) {
        return true;
    }
    return false;
}

void ShipController::runPathfinding(const Map &map, Ship &ship)
{
    // 寻路不成功，设置船状态
    if (!ship.findPath(map)){
        ship.path = Path<VectorPosition>();
        ship.destination = VectorPosition();
        LOGI("尋路失敗",ship.id);
    }
    // 寻路成功，设置船状态
    else{
        LOGI("寻路成功",ship.id);
        ;
    }
}