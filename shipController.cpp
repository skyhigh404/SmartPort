#include "shipController.h"
#include <unordered_set>

// 控制船的整体调度
void ShipController::runController(Map &map,std::vector<Ship> &ships, SeaSingleLaneManager &seaSingleLaneManager){
    // LOGI("shipController::runController");
    // LOGI("ship num:",ships.size());
    auto start = std::chrono::steady_clock::now();
    // 为所有需要寻路算法的船调用寻路算法，给定新目标位置
    // LOGI("进入ship controller");
    for (Ship &ship : ships){
        // LOGI(ship);
        // ship.info();
        // LOGI("是否需要寻路：",needPathfinding(ship));
        // if (ship.state != 0) continue;
        // if (ship.shipStatus != ShipStatusSpace::ShipStatus::MOVING_TO_BERTH &&
        // ship.shipStatus != ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY) 
        //     continue;
        if (needPathfinding(ship)){
            runPathfinding(map, ship);
            // ship.info();
            // for(auto &item : ship.path){
            //     LOGI(item);
            // }
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
    // 尝试次数大于 0 就出错
    for(; tryTime <= 2; ++tryTime){
        reset();
        // 设置船下一帧位置为障碍
        updateTemporaryObstacles(map, ships);
        // 考虑下一步船的行动是否会冲突
        std::set<CollisionEvent, CollisionEventCompare> collisions = detectNextFrameConflict(map, ships, seaSingleLaneManager);
        if(collisions.empty())
            break;

        LOGI("船舶发现冲突");
        for(const auto &collision : collisions) {
            LOGI(collision.shipId1, ", ", collision.shipId2, ", ", collision.type);
        }
        // 遍历冲突船舶集合
        for(const auto &collision : collisions) {
            // 重新规划冲突船的行动以解决冲突
            tryResolveConflict(map,ships, collision);
        }

        // 权衡船舶的规划，设置设定合理的指令
        rePlanShipMove(map, ships);

        map.clearTemporaryObstacles();
        // 直至解决冲突
    }

    // for(const auto &ship : ships)
    //     LOGI(ship);
    end = std::chrono::steady_clock::now();
    countTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if(countTime > 10)
        LOGI("shipController 冲突处理时间: ",countTime," ms, i: ", tryTime);
    // LOGI("shipcontroller执行完毕");
}

bool ShipController::needPathfinding(Ship &ship)
{
    if (ship.shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_BERTH && ship.berthId != -1 && ship.isDestinationValid() && ship.path.empty()) {
        return true;
    }
    if (ship.shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY && ship.isDestinationValid() && ship.path.empty()) {
        return true;
    }
    return false;
}

void ShipController::runPathfinding(const Map &map, Ship &ship)
{
    // 寻路不成功，设置船状态
    if (!ship.findPath(map)){
        ship.path = Path<VectorPosition>();
        // todo 后期是否需要更改目的地
        // ship.destination = VectorPosition();
        LOGE("船寻路失敗",ship);
    }
    // 寻路成功，设置船状态
    else{
        LOGI("船寻路成功: ",ship);
    }
}

void ShipController::tryResolveConflict(Map &map, std::vector<Ship> &ships, const CollisionEvent &event)
{
    // 重新寻路或等待，根据它们的代价来判断，或者往空位走一格
    Ship &ship1 = ships[event.shipId1];
    Ship &ship2 = (event.shipId2 != -1) ? ships[event.shipId2] : ship1; // 用于单一船事件处理
    // 下一帧位置冲突
    if(event.type == CollisionEvent::CollisionType::NextOverlapCollision){
        LOGI("下一帧位置重合冲突");
        // 船 1 的下一帧位置不和船 2 当前位置重合，那么让船 2 等待
        if (!map.hasOverlap(ship1.nextLocAndDir, ship2.locAndDir) && ship1.state != 1){
            // 让船2等待
            LOGI("让船",ship2.id,"等待: ", ship2);
            LOGI("让船",ship1.id, "继续前进: ", ship1);
            makeShipWait(ship2);
        }
        // 船 2 的下一帧位置不和船 1 当前位置重合，那么让船 1 等待
        else if (!map.hasOverlap(ship2.nextLocAndDir, ship1.locAndDir) && ship2.state != 1){
            // 让船1等待
            LOGI("让船",ship1.id, "等待: ", ship1);
            LOGI("让船",ship2.id, "继续前进: ", ship2);
            makeShipWait(ship1);
        }
        // 让优先级低的等待，优先级高的重新寻路
        else if (ship1.comparePriority(map, ship2)){
            // 船2优先级低
            LOGI("让船",ship1.id, "重新寻路，船", ship2.id, "等待");
            makeShipWait(ship2);
            makeShipRefindPath(ship1);
        }
        else {
            LOGI("让船", ship2.id, "重新寻路，船",ship1.id,"等待");
            makeShipWait(ship1);
            makeShipRefindPath(ship2);
        }
        ship1.info();
        ship2.info();
    }
    // 由于指令执行顺序导致的冲突
    else if(event.type == CollisionEvent::CollisionType::PathCrossingCollision){
        LOGI("结算顺序冲突");
        // 有船的下一帧位置不和当前另一个机器人位置重合
        if (!map.hasOverlap(ship1.nextLocAndDir, ship2.locAndDir)){
            // 让船2等待
            LOGI("让船",ship2.id,"等待: ", ship2);
            makeShipWait(ship2);
        }
        else if (!map.hasOverlap(ship2.nextLocAndDir, ship1.locAndDir)){
            // 让船1等待
            LOGI("让船",ship1.id,"等待: ", ship1);
            makeShipWait(ship1);
        }
        else{
            LOGE("冲突解决：结算过程冲突出现意外情况！");
        }
        ship1.info();
        ship2.info();
    }
}

// 根据船的 ShipResolutionActions 权衡合理的规划逻辑
void ShipController::rePlanShipMove(Map &map, std::vector<Ship> &ships){
    std::vector<std::pair<int, ResolutionAction>> refindPathActions; // 存储需要重新寻路的动作及其船舶ID
    // 排序
    for (auto& [key, value] : shipResolutionActions) {
        std::vector<ResolutionAction>& actions = value;
        std::sort(actions.begin(), actions.end(), [](const ResolutionAction& a, const ResolutionAction& b) {
            return a.method < b.method; // 直接根据枚举值的整数比较进行排序,小的优先级高
        });
    }

    // 按优先级最高的规划行动
    // 步骤 1和 2: 遍历所有船舶动作，执行非RefindPath动作，收集RefindPath动作
    for (const auto& [key, value] : shipResolutionActions) {
        Ship &ship = ships.at(key);
        if (value.at(0).method == ResolutionAction::RefindPath) {
            refindPathActions.emplace_back(key, ResolutionAction::RefindPath);
            // refindPathActions.emplace_back(key, value);
        }
        // 等待，去除下一帧障碍，添加当前临时障碍
        else if (value.at(0).method == ResolutionAction::Wait) {
            map.removeTemporaryObstacle(ship.nextLocAndDir);
            stopShip(ship);
            map.addTemporaryObstacle(ship.locAndDir);
        //     LOGI("防止障碍物坐标：", ship.locAndDir);
        //     std::pair<Point2d, Point2d> temp = SpatialUtils::getShipOccupancyRect(ship.locAndDir);
        //     for( int x = temp.first.x; x <= temp.second.x; x++){
        //     for (int y = temp.first.y; y <= temp.second.y; y++){
        //         LOGI(Point2d(x, y), "防止障碍物:", static_cast<int>(map.getCell({x, y})));
        //     }
        // }
        }
        // 重置主航道
        else if (value.at(0).method == ResolutionAction::Dept) {
            ship.shouldDept = true; //让船执行离港指令
        }
    }

    // 加入所有机器人的当前位置作为障碍
    // for (const Ship& ship : ships) {
    //     map.addTemporaryObstacle(ship.pos);
    // }
    
    // if (refindPathActions.size() > 0) {
    //     LOGI("临时障碍位置");
    //     for ( const auto &p : map.temporaryObstacles)
    //         LOGI(p);
    // }

    // 步骤3: 执行所有收集的RefindPath动作
    for (auto& pair : refindPathActions) {
        Ship &ship = ships.at(pair.first);
        LOGI("船舶重新寻路: ", ship);
        map.removeTemporaryObstacle(ship.nextLocAndDir);
        // runPathfinding(map, ship);
        ship.findDetourAndUpdatePath(map);
        ship.updateNextPos();
        LOGI("重新寻路后：", ship);
        map.addTemporaryObstacle(ship.nextLocAndDir);

    }
}

// 让一个船等待
void ShipController::stopShip(Ship &ship){
    ship.nextLocAndDir = ship.locAndDir;
}

// 让一个船离港
void ShipController::deptShip(Ship &ship){
    ship.shouldDept = true;
}

void ShipController::updateTemporaryObstacles(Map &map, std::vector<Ship> &ships)
{
    for (const Ship& ship : ships) {
        map.addTemporaryObstacle(ship.nextLocAndDir);
    }
}

// const Ship & ShipController::decideWhoWaits(Ship &ship1, Ship &ship2){
//     if (ship1.comparePriority(map, ship2))
//         return ship1;
//     else 
//         return ship2;
// }

// 检测船之间是否冲突，输出冲突的船 ID (对)，不考虑地图障碍物的情况
std::set<ShipController::CollisionEvent, ShipController::CollisionEventCompare> 
ShipController::detectNextFrameConflict(Map &map, std::vector<Ship> &ships, SeaSingleLaneManager &seaSingleLaneManager){
    std::set<CollisionEvent, CollisionEventCompare> collision; // 使用 set 保证输出的机器人对不重复
    for(size_t i = 0; i < ships.size(); ++i){
        Ship& ship1 = ships[i];
        // int currentSingleLaneID1 = seaSingleLaneManager.getSingleLaneId(ship1.pos);
        // int nextFrameSingleLaneID1 = seaSingleLaneManager.getSingleLaneId(ship1.nextPos);

        for(size_t j = i + 1; j < ships.size(); ++j){
            Ship& ship2 = ships[j];
            // int currentSingleLaneID2 = seaSingleLaneManager.getSingleLaneId(ship2.pos);
            // int nextFrameSingleLaneID2 = seaSingleLaneManager.getSingleLaneId(ship2.nextPos);

            
            // 判断船下一帧位置冲突
            if (isNextOverlapCollision(map, ship1, ship2)){
                CollisionEvent event(ship1.id, ship2.id ,CollisionEvent::NextOverlapCollision);
                collision.insert(event);
            }
            // 判断结算过程中路径冲突
            if (isPathCrossingCollision(map, ship1, ship2)){
                CollisionEvent event(ship1.id, ship2.id ,CollisionEvent::PathCrossingCollision);
                collision.insert(event);
            }
            // 判断水路单行路冲突
            // todo
        }
    }
    return collision;
}

// 设置标志位，让一个船等待
void ShipController::makeShipWait(const Ship &ship){
    auto it = shipResolutionActions.find(ship.id);
    if (it != shipResolutionActions.end()){
        it->second.push_back(ResolutionAction::Wait);
    } else {
        // 未找到则创建新的向量，并添加解决方案
        shipResolutionActions[ship.id] = std::vector<ResolutionAction>{ResolutionAction::Wait};
    }
}
// 设置标志位，让一个船重新寻路
void ShipController::makeShipRefindPath(const Ship &ship){
    auto it = shipResolutionActions.find(ship.id);
    if (it != shipResolutionActions.end()){
        it->second.push_back(ResolutionAction::RefindPath);
    } else {
        // 未找到则创建新的向量，并添加解决方案
        shipResolutionActions[ship.id] = std::vector<ResolutionAction>{ResolutionAction::RefindPath};
    }
}
// 设置标志位，让一个船移动到临时位置
void ShipController::makeShipDept(const Ship &ship){
    auto it = shipResolutionActions.find(ship.id);
    if (it != shipResolutionActions.end()){
        it->second.push_back(ResolutionAction::Dept);
    } else {
        // 未找到则创建新的向量，并添加解决方案
        shipResolutionActions[ship.id] = std::vector<ResolutionAction>{ResolutionAction::Dept};
    }
}

// // 判断两艘船是否重合，在主航道上的体积不计入，true 是有重叠
// bool ShipController::hasOverlap(Map &map, VectorPosition &a, VectorPosition &b) {
//         std::pair<Point2d, Point2d> ship1 = SpatialUtils::getShipOccupancyRect(a);
//         std::pair<Point2d, Point2d> ship2 = SpatialUtils::getShipOccupancyRect(b);
//         // 检查两个船的矩形是否有重叠部分，计算重叠部分的左上角和右下角坐标
//         int x1 = std::max(ship1.first.x, ship2.first.x);
//         int y1 = std::max(ship1.first.y, ship2.first.y);
//         int x2 = std::min(ship1.second.x, ship2.second.x);
//         int y2 = std::min(ship1.second.y, ship2.second.y);

//         // 检查是否有重叠部分
//         bool overlap = (x1 <= x2) && (y1 <= y2);
//         if (overlap) {
//             // 如果重叠部分全部位于主航道，则不算重合
//             for( int x = x1; x <= x2; x++){
//                 for (int y = y1; y <= y2; y++){
//                     if (!map.isInSealane({x, y}))
//                         return true;
//                 }
//             }
//             // 全部位于主航道
//             return false;
//         } 
//         return false;
// }



// 判断两艘船下一帧位置是否冲突
bool ShipController::isNextOverlapCollision(Map &map, Ship &a, Ship &b){
    return map.hasOverlap(a.nextLocAndDir, b.nextLocAndDir);
}

// 判断两艘船是否是由于执行顺序而引起的冲突
bool ShipController::isPathCrossingCollision(Map &map, Ship &a, Ship &b){
    if (isNextOverlapCollision(map, a, b)) 
        return false;
    // 优先级高的船下一帧位置和优先级低的船当前位置产生了重合
    if (a.id < b.id) 
        return map.hasOverlap(a.nextLocAndDir, b.locAndDir);
    else 
        return map.hasOverlap(a.locAndDir, b.nextLocAndDir);
}
