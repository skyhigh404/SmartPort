#include "greedyShipScheduler.h"

//  设置参数
void GreedyShipScheduler::setParameter(const Params &params)
{
    ABLE_DEPART_SCALE = params.ABLE_DEPART_SCALE;
    MAX_SHIP_NUM = params.MAX_SHIP_NUM;
    TIME_TO_WAIT = params.TIME_TO_WAIT;
    CAPACITY_GAP = params.CAPACITY_GAP;
}

void GreedyShipScheduler::scheduleShips(Map &map, std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods, std::vector<Robot> &robots) {
    //需要迁移，更新泊位和货物的状态
    updateBerthStatus(ships, berths, goods);

    // 2. 决定调度策略
    std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> actions;
    ShipActionSpace::ShipAction action;
    for(auto &ship : ships){
        switch (ship.state) {
        case 0: // 在路途中 | 在交货点
            handleShipOnRoute(map, ship, berths, goods);
            break;
        case 1: // 在恢复状态
        // action = ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,-1); 
            break;
        case 2: // 装载状态 && 在泊位
            handleShipAtBerth(map, ship, berths, goods);
            // action = handleShipWaiting(ship, berths, goods);
            break;
        }
        // // 解决冲突情况
        // actions.push_back(std::make_pair(ship.id,action));
    }
    // return actions;
}

// 处理船在路途的情况
void GreedyShipScheduler::handleShipOnRoute(Map& map, Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    BerthID berthId = ship.berthId;
    // 船是空闲状态
    if (ship.isIdle()){
        BerthID berthId = findBestBerthForShip(map, ship, berths, goods);
        ship.updateMoveToBerthStatus(berthId, VectorPosition(berths[berthId].pos, Direction::EAST));
        // return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH,berthId); 
    }

    //  如果路径不为空
    if (!ship.path.empty()){
    }
    // 路径为空且到达泊位
    else if (ship.path.empty() && ship.reachBerth()){
        ship.updateLoadStatus();
    }
    // 路径为空且到达交货点
    else if (ship.path.empty() && ship.reachDelivery()){
        BerthID berthId = findBestBerthForShip(map, ship, berths, goods);
        ship.updateMoveToBerthStatus(berthId, VectorPosition(berths[berthId].pos, Direction::EAST));
        // return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH,berthId);
    }
    else{
        // 路径不为空就到达目的地
        LOGE("船路径不为空，但抵达目的地");
    }
}

// 处理船在泊位上的情况
void GreedyShipScheduler::handleShipAtBerth(Map &map, Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 分配交货点id
    int deliveryId = allocateDelivery(berths[ship.berthId]);
    // 前往交货点
    if(shouldDepartBerth(ship, berths)){
        ship.updateMoveToDeliveryStatus(VectorPosition(map.deliveryLocations[deliveryId], Direction::EAST));
        // return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_DELIVERY,deliveryId);
    }
    // 有货转货
    else if(isThereGoodsToLoad(berths[ship.berthId])){
        BerthID berthId = ship.berthId;
        int shipment = std::min(static_cast<int>(berths[berthId].reached_goods.size()),berths[berthId].velocity);
        int res = ship.loadGoods(shipment); // 装货
        berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);
        return;
    }
    // 容量不多，考虑是否直接去虚拟点
    else if(ship.capacityScale() < ABLE_DEPART_SCALE){
        // 附件有货，再等等
        if (isGoodsArrivingSoon(berths[ship.berthId], goods)) return;
        // 附件没货，直接去送货点
        else {
            ship.updateMoveToDeliveryStatus(VectorPosition(map.deliveryLocations[deliveryId], Direction::EAST));
        }
    }
    // 容量还多，等待分配泊位
    else{
        BerthID berthId = findBestBerthForShip(map, ship, berths, goods);
        if(berthId != ship.berthId && berthId != -1) {
            ship.updateMoveToBerthStatus(berthId, VectorPosition(berths[berthId].pos, Direction::EAST));
        }
    }

}

// // 处理船在虚拟点的情况
// ShipActionSpace::ShipAction
// GreedyShipScheduler::handleShipInEnd(Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
//     // 寻找当前最优的泊位
//     BerthID berthId = findBestBerthForShip(ship, berths, goods);
//     #ifdef DEBUG
//     assert(berthId != -1);
//     #endif
//     if(berthId != -1){
//         return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, berthId);
//     }
//     else{
//         return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, berthId);
//         #ifdef DEBUG
//         LOGE("报错，在虚拟点的船找不到目标泊位");
//         ship.info();
//         #endif
//     }
// }

// // 处理在泊位外等待的情况
// ShipActionSpace::ShipAction
// GreedyShipScheduler::handleShipWaiting( Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
//     BerthID berthId = findBestBerthForShip(ship, berths, goods);
//     // 调度失败
//     if(berthId == -1 || berthId == ship.berthId) return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,ship.berthId);
//     // 调度泊位溢出货物量比当前泊位
//     int bestCanLoadNum = std::min(berths[berthId].residue_num, ship.nowCapacity());  // 调度泊位中能装的货物数量
//     int nowCanLoadNum = std::min(berths[ship.berthId].residue_num + ship.nowCapacity(), ship.nowCapacity());  // 当前泊位能装的货物数量
//     // 调度泊位价值和需求 大于 当前泊位
//     // todo 超参数
//     if(berths[berthId].totalValue > berths[ship.berthId].totalValue && bestCanLoadNum > nowCanLoadNum * 1.5){
//         return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, berthId);
//     }
//     // 保持不动
//     else{
//         return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, ship.berthId);
//     }
// }


// 初始化泊位的状态
void GreedyShipScheduler::updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths,std::vector<Goods> & goods){
    // 遍历泊位，初始化泊位的正常货物量和未到达货物量，计算泊位价值
    for(auto &berth : berths){
        berth.totalValue = 0;
        berth.residue_num = 0;
        berth.shipInBerthNum = 0;
        berth.residue_num = berth.reached_goods.size();
    }
    // 遍历船只，更新前泊位上的船只数量和更新溢出货物量
    for(auto &ship : ships){
        if (ship.berthId != -1){
            berths[ship.berthId].residue_num -= ship.nowCapacity();
            berths[ship.berthId].shipInBerthNum += 1;
        }
    }
    // 遍历泊位，累加当前溢出货物价值
    for(auto &berth : berths){
        for(int index = berth.reached_goods.size() - berth.residue_num; index < berth.reached_goods.size(); index++){
            berth.totalValue += berth.reached_goods[index].value;
        }
    }
    //  遍历货物，更新泊位的溢出货物量和价值
    // todo 根据货物到达泊位的距离进行加权影响
    for(auto &good : goods){
        // todo 后续需要考虑泊位禁用情况
        if(good.status == 1 || good.status == 2){
             berths[good.distsToBerths[0].first].totalValue += calculateGoodValueByDist(good);
             berths[good.distsToBerths[0].first].residue_num += 1;
        }
    }
}

// 根据货物距离泊位距离计算货物价值(选取最短距离)
float GreedyShipScheduler::calculateGoodValueByDist(Goods &good){
    // 过期前无法到达则价值为0
    if(good.distsToBerths[0].second > good.TTL) return 0;

    // todo 设置超参，超过 distLimit 范围货物价值不予考虑
    int distLimit = 500;
    float beta = -1 / distLimit * good.distsToBerths[0].second + 1;
    beta = std::max(beta,static_cast<float>(0.0));

    return beta * good.value;
}

// 判断船只是否需要前往虚拟点
bool GreedyShipScheduler::shouldDepartBerth( Ship &ship,std::vector<Berth> &berths){
    // 1. 船满了，前往虚拟点
    if (ship.nowCapacity() <= 0)  return true;
    // 2. 游戏快结束了，前往虚拟点
    // todo 15000改成全局变量；缓冲时间变成超参
    // 泊位交货点时间
    // int timeToDeliveryLocation = berths[ship.berthId].
    else if(ship.berthId != -1 && 15000 - CURRENT_FRAME <= berths[ship.berthId].timeToDelivery() + 2) return true;
    else return false;
}


// 判断泊位上是否有货物可装载
bool GreedyShipScheduler::isThereGoodsToLoad(Berth &berth){
    if(berth.reached_goods.size() != 0) return true;
    else return false;
}

// 判断泊位最近有没有货物到来
bool GreedyShipScheduler::isGoodsArrivingSoon(Berth &berth, std::vector<Goods> goods){
    // todo 15000后期修改成超参数
    int timeToWait = std::min(TIME_TO_WAIT, 15000 - CURRENT_FRAME - berth.timeToDelivery() - 5);
    for(auto good : goods){
        if((good.status == 1 || good.status == 2)
        && good.distsToBerths[0].first == berth.id
        && good.distsToBerths[0].second <= timeToWait
        && good.distsToBerths[0].second <= good.TTL){
            return true;
        }
    }
    return false;
}

// 为船找到最佳泊位，返回泊位id（局部最优）
// 考虑：泊位前往虚拟点的时间代价、泊位未来的收益、泊位货物量和船容量的匹配
BerthID GreedyShipScheduler::findBestBerthForShip(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods){
    // 泊位进行收益排序(将来一段时间)
    // std::vector<Berth> berths_sort(berths);
    // int currentFrame = CURRENT_FRAME;
    // std::sort(berths_sort.begin(), berths_sort.end(),[ship](Berth a,Berth berths){
    //     // 如果剩余时间不够,则优先级最低
    // });
    // todo 考虑泊位运输虚拟点时间的代价
    int absCapacity = 0;
    int highestValue = 0;
    BerthID assignBerthId = -1;
    for(auto &berth : berths){
        if(canMoveBerth(map, ship, berth) && berth.shipInBerthNum < MAX_SHIP_NUM){
            if(berth.totalValue > highestValue){
                if(highestValue == 0){
                    assignBerthId = berth.id;
                    highestValue = berth.totalValue;
                    absCapacity = std::abs(berth.residue_num - ship.nowCapacity());
                }
                // todo 容量匹配优先
                else if(std::abs(berth.residue_num - ship.nowCapacity()) < absCapacity){
                    assignBerthId = berth.id;
                    highestValue = berth.totalValue;
                    absCapacity = std::abs(berth.residue_num - ship.nowCapacity());
                }
            }
        }
    }
    return assignBerthId;
}

// 判断船可以前往其他泊位
bool GreedyShipScheduler::canMoveBerth(Map &map, Ship &ship,Berth &berth){
    // 相同泊位返回true
    if (ship.berthId == berth.id) return true;

    // 如果船前往其他泊位后还有时间前往虚拟点，则返回true
    // todo 泊位移动距离和缓冲时间改成超参
    int timeCost = berth.timeToDelivery() + static_cast<int>(ship.nowCapacity() / berth.velocity);
    // 船在交货点
    if(ship.reachDelivery()) timeCost += berth.timeToDelivery();
    // 船在泊位上，并且目的地不同
    else if(ship.berthId != -1){
        // todo maritimeBerthDistanceMap包括泊位到泊位吗
        Point2d targetBerthPos = berth.pos;
        int timeToBerth = map.maritimeBerthDistanceMap[ship.id][targetBerthPos.x][targetBerthPos.y];
        timeCost += timeToBerth;
    } 

    if (CURRENT_FRAME + timeCost + 10<= 15000) return true;
    else return false;
}

// 当船从当前泊位移动到其他泊位时，更新泊位相关参数
void GreedyShipScheduler::updateBerthWhereShipMove(Ship &ship,std::vector<Berth> &berths,BerthID targetId){
    // 原泊位状态更新
    BerthID nowBerthId = ship.berthId;
    berths[nowBerthId].shipInBerthNum -= 1;
    berths[nowBerthId].residue_num += ship.nowCapacity();
    int startIndex = berths[nowBerthId].reached_goods.size() - berths[nowBerthId].residue_num;
    for(int index = startIndex;index < berths[nowBerthId].reached_goods.size() && index < startIndex + ship.nowCapacity(); index++){
        berths[nowBerthId].totalValue += berths[nowBerthId].reached_goods[index].value;
    }
    // 目标泊位状态更新
    berths[targetId].shipInBerthNum += 1;
    berths[targetId].residue_num -= ship.nowCapacity();
    startIndex = berths[targetId].reached_goods.size() - berths[targetId].residue_num - ship.nowCapacity();
    for(int index = startIndex;index < berths[targetId].reached_goods.size() && index < startIndex + ship.nowCapacity(); index++){
        berths[targetId].totalValue -= berths[targetId].reached_goods[index].value;
    }
    #ifdef DEBUG
    assert(berths[nowBerthId].shipInBerthNum >= 0);
    assert(berths[targetId].shipInBerthNum >= 0);
    #endif
}


// 获取最近的交货点
//  todo 后续和timeToDelivery统一起来
int GreedyShipScheduler::allocateDelivery( Berth &berth){
    int deliveryId = berth.distsToDelivery[0].first;
    return deliveryId;
}