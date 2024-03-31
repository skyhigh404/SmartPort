#include "greedyShipScheduler.h"

//  设置参数
void GreedyShipScheduler::setParameter(const Params &params)
{
    ABLE_DEPART_SCALE = params.ABLE_DEPART_SCALE;
    MAX_SHIP_NUM = params.MAX_SHIP_NUM;
    TIME_TO_WAIT = params.TIME_TO_WAIT;
    CAPACITY_GAP = params.CAPACITY_GAP;
}

std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> GreedyShipScheduler::scheduleShips(Map &map, std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods, std::vector<Robot> &robots) {
    // 1. 更新泊位和货物的状态
    updateBerthStatus(ships, berths, goods);

    // 2. 根据船只的状态决定策略
    std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> actions;
    for (auto &ship : ships) {
        ShipActionSpace::ShipAction action;
        switch (ship.state) {
            case 0: // 在路途中（如果前往泊位途中，berthId为该泊位id）
                action = handleShipOnRoute(ship, berths,goods);
                break;
            case 1: // 在泊位上 | 在虚拟点
                if(ship.berthId != -1){
                    action = handleShipAtBerth(ship, berths, goods);
                }
                else{
                    action = handleShipInEnd(ship, berths, goods);
                }
                break;
            case 2: // 在泊位外等待
                action = handleShipWaiting(ship, berths, goods);
                break;
        }
        // 解决冲突情况
        actions.push_back(std::make_pair(ship.id,action));
    }

    return actions;
}

// 处理船在路途的情况
// 是否可以中途前往其他泊位（收益更高）
ShipActionSpace::ShipAction
GreedyShipScheduler::handleShipOnRoute(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 暂时不做调度
    BerthID berthId = ship.berthId;
    return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,berthId); 
}

// 处理船在泊位上的情况
ShipActionSpace::ShipAction
GreedyShipScheduler::handleShipAtBerth(Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 前往虚拟点
    if(shouldDepartBerth(ship, berths)){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH,ship.berthId);
    }
    // 有货转货
    else if(isThereGoodsToLoad(berths[ship.berthId])){
        BerthID berthId = ship.berthId;
        int shipment = std::min(static_cast<int>(berths[berthId].reached_goods.size()),berths[berthId].velocity);
        int res = ship.loadGoods(shipment); // 装货
        berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,berthId);
    }
    // 容量不多，考虑是否直接去虚拟点
    else if(ship.capacityScale() < ABLE_DEPART_SCALE){
        // 附件有货，再等等
        if (isGoodsArrivingSoon(berths[ship.berthId], goods)) return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,ship.berthId);
        // 附件没货，直接去虚拟点
        else return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH,ship.berthId);
    }
    // 容量还多，等待分配泊位
    else{
        BerthID berthId = findBestBerthForShip(ship, berths, goods);
        if(berthId != ship.berthId && berthId != -1) {
            return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, berthId);
        }
        else return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, berthId);
    }

}

// 处理船在虚拟点的情况
ShipActionSpace::ShipAction
GreedyShipScheduler::handleShipInEnd(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 寻找当前最优的泊位
    BerthID berthId = findBestBerthForShip(ship, berths, goods);
    #ifdef DEBUG
    assert(berthId != -1);
    #endif
    if(berthId != -1){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, berthId);
    }
    else{
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, berthId);
        #ifdef DEBUG
        LOGE("报错，在虚拟点的船找不到目标泊位");
        ship.info();
        #endif
    }
}

// 处理在泊位外等待的情况
ShipActionSpace::ShipAction
GreedyShipScheduler::handleShipWaiting(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    BerthID berthId = findBestBerthForShip(ship, berths, goods);
    // 调度失败
    if(berthId == -1 || berthId == ship.berthId) return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,ship.berthId);
    // 调度泊位溢出货物量比当前泊位
    int bestCanLoadNum = std::min(berths[berthId].residue_num, ship.now_capacity);  // 调度泊位中能装的货物数量
    int nowCanLoadNum = std::min(berths[ship.berthId].residue_num + ship.now_capacity, ship.now_capacity);  // 当前泊位能装的货物数量
    // 调度泊位价值和需求 大于 当前泊位
    // todo 超参数
    if(berths[berthId].totalValue > berths[ship.berthId].totalValue && bestCanLoadNum > nowCanLoadNum * 1.5){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, berthId);
    }
    // 保持不动
    else{
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, ship.berthId);
    }
}


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
            berths[ship.berthId].residue_num -= ship.now_capacity;
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
bool GreedyShipScheduler::shouldDepartBerth(const Ship &ship,std::vector<Berth> &berths){
    // 1. 船满了，前往虚拟点
    if (ship.now_capacity <= 0)  return true;
    // 2. 游戏快结束了，前往虚拟点
    // todo 15000改成全局变量；缓冲时间变成超参
    else if(ship.berthId != -1 && 15000 - CURRENT_FRAME <= berths[ship.berthId].time + 2) return true;
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
    int timeToWait = std::min(TIME_TO_WAIT, 15000 - CURRENT_FRAME - berth.time - 5);
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
BerthID GreedyShipScheduler::findBestBerthForShip(const Ship &ship, const std::vector<Berth> &berths, const std::vector<Goods> &goods){
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
        if(canMoveBerth(ship, berth) && berth.shipInBerthNum < MAX_SHIP_NUM){
            if(berth.totalValue > highestValue){
                if(highestValue == 0){
                    assignBerthId = berth.id;
                    highestValue = berth.totalValue;
                    absCapacity = std::abs(berth.residue_num - ship.now_capacity);
                }
                // todo 容量匹配优先
                else if(std::abs(berth.residue_num - ship.now_capacity) < absCapacity){
                    assignBerthId = berth.id;
                    highestValue = berth.totalValue;
                    absCapacity = std::abs(berth.residue_num - ship.now_capacity);
                }
            }
        }
    }
    return assignBerthId;
}

// 判断船可以前往其他泊位
bool GreedyShipScheduler::canMoveBerth(const Ship &ship,const Berth &berth){
    // 路途中的船暂时不考虑调度
    if(ship.state == 0) return false;
    // 相同泊位返回true
    if (ship.berthId == berth.id) return true;

    // 如果船前往其他泊位后还有时间前往虚拟点，则返回true
    // todo 泊位移动距离和缓冲时间改成超参
    int timeCost = berth.time + static_cast<int>(ship.now_capacity / berth.velocity);
    // 船在虚拟点
    if(ship.state == 1 && ship.berthId == -1) timeCost += berth.time;
    // 船在泊位上，并且目的地不同
    else if(ship.state == 1 || ship.state == 2) timeCost += 500;

    if (CURRENT_FRAME + timeCost + 2<= 15000) return true;
    else return false;
}

// 当船从当前泊位移动到其他泊位时，更新泊位相关参数
void GreedyShipScheduler::updateBerthWhereShipMove(Ship &ship,std::vector<Berth> &berths,BerthID targetId){
    // 原泊位状态更新
    BerthID nowBerthId = ship.berthId;
    berths[nowBerthId].shipInBerthNum -= 1;
    berths[nowBerthId].residue_num += ship.now_capacity;
    int startIndex = berths[nowBerthId].reached_goods.size() - berths[nowBerthId].residue_num;
    for(int index = startIndex;index < berths[nowBerthId].reached_goods.size() && index < startIndex + ship.now_capacity; index++){
        berths[nowBerthId].totalValue += berths[nowBerthId].reached_goods[index].value;
    }
    // 目标泊位状态更新
    berths[targetId].shipInBerthNum += 1;
    berths[targetId].residue_num -= ship.now_capacity;
    startIndex = berths[targetId].reached_goods.size() - berths[targetId].residue_num - ship.now_capacity;
    for(int index = startIndex;index < berths[targetId].reached_goods.size() && index < startIndex + ship.now_capacity; index++){
        berths[targetId].totalValue -= berths[targetId].reached_goods[index].value;
    }
    #ifdef DEBUG
    assert(berths[nowBerthId].shipInBerthNum >= 0);
    assert(berths[targetId].shipInBerthNum >= 0);
    #endif
}