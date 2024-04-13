#include "greedyShipScheduler.h"

//  设置参数
void GreedyShipScheduler::setParameter(const Params &params)
{
    ABLE_DEPART_SCALE = params.ABLE_DEPART_SCALE;
    MAX_SHIP_NUM = params.MAX_SHIP_NUM;
    TIME_TO_WAIT = params.TIME_TO_WAIT;
    CAPACITY_GAP = params.CAPACITY_GAP;
    SHIP_WAIT_TIME_LIMIT = params.SHIP_WAIT_TIME_LIMIT;
    GOOD_DISTANCE_LIMIT = params.GOOD_DISTANCE_LIMIT;
    DELIVERY_VALUE_LIMIE = params.DELIVERY_VALUE_LIMIE;
    EARLY_DELIVERT_FRAME_LIMIT = params.EARLY_DELIVERT_FRAME_LIMIT;
    EARLY_DELIVERY_VALUE_LIMIT = params.EARLY_DELIVERY_VALUE_LIMIT;
}

void GreedyShipScheduler::scheduleShips(Map &map, std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods, std::vector<Robot> &robots) {
    //需要迁移，更新泊位和货物的状态
    updateBerthStatus(ships, berths, goods, robots);
    // LOGI("测试当前全局参数：", CURRENT_FRAME, ", ", FINAL_FRAME, ", ", CURRENT_MONEY, ", ", static_cast<int>(MAP_TYPE));
    // LOGI("初始化泊位状态完毕");

    // 2. 决定调度策略
    std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> actions;
    ShipActionSpace::ShipAction action;
    // LOGI("scheduler ship num：",ships.size());
    for(auto &ship : ships){
        // LOGI("进入贪心船调度");
        // ship.info();
        switch (ship.state) {
        case 0: // 在路途中 | 在交货点
            handleShipOnRoute(map, ship, berths, goods);
            break;
        case 1: // 在恢复状态
        // action = ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE,-1); 
            break;
        case 2: // 装载状态 && 在泊位
            // 前往交货点
            if (ship.isMoveToDelivery() && !ship.path.empty()) continue;
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
    // LOGI("handleShipOnRoute");
    // ship.info();

    // 船是空闲状态
    if (ship.isIdle()){
        LOGI("船舶空闲状态：", ship.id);
        scheduleShipAtShipShops(map, ship, berths, goods);
    }

    //  如果路径不为空
    if (!ship.path.empty()){
        return;
    }
    // 路径为空且到达泊位
    else if (ship.path.empty() && ship.reachBerth()){
        LOGI("到达泊位");
        ship.updateLoadStatus();
    }
    // 路径为空且到达交货点
    else if (ship.path.empty() && ship.reachDelivery()){
        LOGI("到达交货点");
        scheduleShipAtDelivery(map, ship, berths, goods);
    }
    // else{
    //     // 路径为空，但没有抵达目的地
    //     LOGE("船路径为空，没有抵达目的地");
    //     ship.info();
    // }
}

// 处理船在泊位上的情况
void GreedyShipScheduler::handleShipAtBerth(Map &map, Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    LOGI("船在泊位上");
    ship.info();
    // 分配交货点id
    int deliveryId = allocateDelivery(berths[ship.berthId]);
    // 前往交货点
    if(shouldDepartBerth(ship, berths)){
        LOGI("应该前往交货点");
        ship.info();
        ship.updateMoveToDeliveryStatus(deliveryId, VectorPosition(map.deliveryLocations[deliveryId], Direction::EAST));
        berths[ship.berthId].shipInBerthNum = std::max(0, berths[ship.berthId].shipInBerthNum - 1);
        berths[ship.berthId].info();
        // return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_DELIVERY,deliveryId);
    }
    // 有货转货
    else if(isThereGoodsToLoad(berths[ship.berthId])){
        // 早期船赚够钱直接出发
        if (mustShipDepartEarly(ship)){
            ship.updateMoveToDeliveryStatus(deliveryId, VectorPosition(map.deliveryLocations[deliveryId], Direction::EAST));
            berths[ship.berthId].shipInBerthNum = std::max(0, berths[ship.berthId].shipInBerthNum - 1);
        }

        BerthID berthId = ship.berthId;
        int shipment = std::min(static_cast<int>(berths[berthId].reached_goods.size()),berths[berthId].velocity);
        int res = ship.loadGoods(shipment); // 装货
        // 累计货物的价值
        for (int index = 0; index < res; index++){
            ship.loadGoodValue += berths[berthId].reached_goods[index].value;
        }
        berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);
        LOGI("装货中-----------------------------");
        LOGI("装货数量：", res);
        ship.info();
        berths[ship.berthId].info();
        Berth::maxLoadGoodNum += res;
        return;
    }
    // 附近没有货物
    // 用意：不让船动的太频繁
    // else if(!isThereGoodsToLoadRecently(berths[ship.berthId], goods)){
    //     // 衡量船去每一个泊位和直接去交货点的收益
    //     scheduleFreeShipAtBerth(map, ship, berths, goods);
    // }
    else{
        scheduleFreeShipAtBerth(map, ship, berths, goods);
    }

}

// 初始化泊位的状态
void GreedyShipScheduler::updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths,std::vector<Goods> & goods, std::vector<Robot> &robots){
    // 遍历泊位，初始化泊位的正常货物量和未到达货物量，计算泊位价值
    for(auto &berth : berths){
        berth.totalValue = 0;
        berth.residue_num = 0;
        berth.residue_value = 0;
        berth.shipInBerthNum = 0;
        berth.futureValue = 0;
        berth.onRouteTime = INT_MAX;
        berth.residue_num = berth.reached_goods.size();
    }
    // 遍历船只，更新前泊位上的船只数量和更新溢出货物量
    for(auto &ship : ships){
        // todo 考虑是否计算未分配（状态0）的货物价值
        if (ship.berthId != -1 && ship.shipStatus != ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY){
            berths[ship.berthId].residue_num -= ship.nowCapacity();
            berths[ship.berthId].shipInBerthNum += 1;
            // 记录船前往泊位上的时间
            if (ship.shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_BERTH) berths[ship.berthId].onRouteTime = std::min(static_cast<int>(ship.path.size()), berths[ship.berthId].onRouteTime);
        }
        if(berths[ship.berthId].onRouteTime == INT_MAX) berths[ship.berthId].onRouteTime = 0;
    }
    // 遍历泊位，累加当前溢出货物价值
    for(auto &berth : berths){
        int reachGoodsSize = static_cast<int>(berth.reached_goods.size());
        for(int index = 0; index < reachGoodsSize - berth.residue_num && index < reachGoodsSize; index++){
            berth.totalValue += berth.reached_goods[index].value;
        }
        for(int index = reachGoodsSize - berth.residue_num; index < reachGoodsSize && index >= 0; index++){
            berth.totalValue += berth.reached_goods[index].value;
            berth.residue_value += berth.reached_goods[index].value;
        }
    }
    // 终局前
    if (CURRENT_FRAME < FINAL_FRAME){
        //  遍历货物，更新泊位的未到达货物的价值，用来估算泊位未来价值futureValue
        // todo 根据货物到达泊位的距离进行加权影响
        for(auto &good: goods){
            // 已分配的货物（状态1），根据距离选泊位； 
            if (good.status == 1 && good.distsToBerths[0].second <= GOOD_DISTANCE_LIMIT && good.TTL >= GOOD_DISTANCE_LIMIT){
                berths[good.distsToBerths[0].first].futureValue += calculateGoodValueByDist(good);
            }
        }
        for (auto &robot: robots){
            // 累加运送途中的货物价值
            if (robot.type==0 && robot.carryingItem == 1 && robot.carryingItemId != -1 && robot.targetid != -1){
                berths[robot.targetid].futureValue += calculateGoodValueByDist(goods[robot.carryingItemId]);
            }
            if (robot.type==1 && robot.carryingItem == 2 && robot.carryingItemId != -1 && robot.carryingItemId2 != -1 && robot.targetid != -1) {
                berths[robot.targetid].futureValue += calculateGoodValueByDist(goods[robot.carryingItemId]);
                berths[robot.targetid].futureValue += calculateGoodValueByDist(goods[robot.carryingItemId2]);
            }
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
    else if(ship.berthId != -1 && std::abs(CURRENT_FRAME + berths[ship.berthId].timeToDelivery() - 15000) <= 25) return true;
    else return false;
}

// 判断泊位上是否有货物可装载
bool GreedyShipScheduler::isThereGoodsToLoad(Berth &berth){
    if(berth.reached_goods.size() != 0) return true;
    else return false;
}

// 判断泊位上短时间内是否有货物可以状态
bool GreedyShipScheduler::isThereGoodsToLoadRecently(Berth &berth, std::vector<Goods> &goods){
    //  遍历货物，判断SHIP_WAIT_TIME_LIMIT时间内有没有货物到达
    int value = 0;
    for(auto &good : goods){
        if((good.status == 1 || good.status == 2)
        && good.distsToBerths[0].first == berth.id
        && good.distsToBerths[0].second <= SHIP_WAIT_TIME_LIMIT
        && good.distsToBerths[0].second <= good.TTL
        && good.TTL != INT_MAX){
            value += good.value;
        }
    }
    // todo 后期可以设置超参数
    if(value > 500) {
        LOGI("最近", SHIP_WAIT_TIME_LIMIT, "帧内货物价值：", value);
        berth.info();
        return true;
    }
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

// 当船在虚拟点时，选择最佳调度策略
// todo 后续要判断时间是否足够（排序）
void GreedyShipScheduler::scheduleShipAtDelivery(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods){
    std::vector<std::pair<BerthID, float>> profitBerths;
    
    if (ship.deliveryId == -1){
        LOGE("scheduleShipAtDelivery：报错，船的交货点id为-1");
        return;
    }

    for (auto &berth: berths){
        // 收益 / 距离
        int distance = map.berthToDeliveryDistance[berth.id][ship.deliveryId];
        auto profit = calculateShipProfitInBerth(map, ship, berth);
        profitBerths.push_back({berth.id, 1.0 * profit.first/(profit.second + distance)});
    }

    // 综合收益进行排序
    std::sort(profitBerths.begin(), profitBerths.end(), [](std::pair<BerthID, float> &a, std::pair<BerthID, float> &b){
        return a.second > b.second;
    });

    // 选取最佳泊位
    std::pair<BerthID, float> bestBerthAndProfit = {-1, 0};
    for (auto &profitBerth : profitBerths){
        // if (berths[profitBerth.first].shipInBerthNum < MAX_SHIP_NUM){
        if (canShipMoveToBerth(map, ship, berths[profitBerth.first])){
            bestBerthAndProfit = profitBerth;
            break;
        }
    }

    // 泊位选取有误 && 进去终局前报错
    if (bestBerthAndProfit.first == -1 && CURRENT_FRAME < FINAL_FRAME){
        LOGE("scheduleShipAtDelivery：泊位分配有误，每个泊位上都有船");
        ship.info();
        return;
    }

    ship.updateMoveToBerthStatus(bestBerthAndProfit.first, VectorPosition(berths[bestBerthAndProfit.first].pos, berths[bestBerthAndProfit.first].orientation));
    LOGI("分配泊位id:", bestBerthAndProfit.first);
}

// 当船在泊位时（没货），选择最佳调度策略（去泊位|去虚拟点）
// todo 后续要判断时间是否足够(排序)
void GreedyShipScheduler::scheduleFreeShipAtBerth(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods){
    std::vector<std::pair<BerthID, float>> profitBerths;
    std::vector<std::pair<int, float>> profitDelivery;  //第一维是交货点id，第二维是去交货点收益

    if (ship.berthId == -1){
        LOGE("scheduleShipAtBerth：报错，船的泊位id为-1");
        return;
    }
    
    // 1. 前往另一个泊位再去虚拟点的收益
    // 计算每个泊位的收益
    for (auto &berth: berths){
        // 收益 / 距离
        int distance = map.berthToBerthDistance[berth.id][ship.berthId];
        auto profitBerth = calculateShipProfitInBerth(map, ship, berth);
        // （未来装货价值 + 当前船价值） / (装货时间 + 前往目标泊位距离 + 目标泊位前往虚拟点距离)
        float profit = (1.0 * profitBerth.first + ship.loadGoodValue) /(profitBerth.second + distance + berth.distsToDelivery[0].second);
        profitBerths.push_back({berth.id, profit});
    }

    // 综合收益进行排序
    std::sort(profitBerths.begin(), profitBerths.end(), [](std::pair<BerthID, float> &a, std::pair<BerthID, float> &b){
        return a.second > b.second;
    });

    // 选取最佳泊位
    std::pair<BerthID, float> bestBerthAndProfit = {-1, 0};
    for (auto &profitBerth : profitBerths){
        // todo 泊位可用判断 (用于终局调度时刻)，判断是否有必要添加
        if (canShipMoveToBerth(map, ship, berths[profitBerth.first]) && berths[profitBerth.first].isEnable()){
            bestBerthAndProfit = profitBerth;
            break;
        }
    }

    // 泊位选取有误
    if (bestBerthAndProfit.first == -1){
        // 进入终局时刻，直接找收益最高的泊位
        if (CURRENT_FRAME > FINAL_FRAME){
            bestBerthAndProfit = profitBerths[0];
        }
        // 正常时刻找不到泊位
        else {
        LOGE("scheduleFreeShipAtBerth：泊位分配有误，每个泊位上都有船");
        ship.info();
        }
    }

    //2. 直接去虚拟点的收益
    int deliveryId = allocateDelivery(berths[ship.berthId]);
    int deliveryProfit = 1.0 * ship.loadGoodValue / map.berthToDeliveryDistance[ship.berthId][deliveryId];

    LOGI("最佳泊位收益：", bestBerthAndProfit.second );
    berths[bestBerthAndProfit.first].info();

    LOGI("最佳虚拟点收益：", deliveryProfit, "耗时：", map.berthToDeliveryDistance[ship.berthId][deliveryId]);

    // 去泊位收益最高 || 当前金额不够买一个机器人
    // todo 不去虚拟点的条件应该进行多次调参 || 可以限制只有当运输价值大于多少时才去虚拟点
    // if(bestBerthAndProfit.second > deliveryProfit || (ship.loadGoodValue < DELIVERY_VALUE_LIMIE || ship.loadGoodValue + CURRENT_MONEY < 2000)){
    if((bestBerthAndProfit.second > deliveryProfit && !(mustShipDepartEarly(ship))) || deliveryProfit == 0){
        if (bestBerthAndProfit.first == ship.berthId && CURRENT_FRAME < FINAL_FRAME){
            LOGE("船选中相同泊位进行移动！");
            ship.info();
        }
        ship.updateMoveToBerthStatus(bestBerthAndProfit.first, VectorPosition(berths[bestBerthAndProfit.first].pos, berths[bestBerthAndProfit.first].orientation));
        updateBerthWhenShipMove(ship, berths, bestBerthAndProfit.first);
        LOGI("去泊位收益更高:", ship);
    }
    // 去虚拟点收益最高
    else{
        ship.updateMoveToDeliveryStatus(deliveryId, VectorPosition(map.deliveryLocations[deliveryId], Direction::EAST));
        berths[ship.berthId].shipInBerthNum = std::max(0, berths[ship.berthId].shipInBerthNum - 1);
        LOGI("去虚拟点收益更高：", ship);
        // berths[ship.berthId].info();
    }
}

// 当船在购买点时
void GreedyShipScheduler::scheduleShipAtShipShops(Map& map, Ship &ship, std::vector<Berth> &berths, const std::vector<Goods> &goods){
    std::vector<std::pair<BerthID, float>> profitBerths; // 第一维是泊位id，第二维是收益
    for (auto &berth : berths){
        int distance = map.maritimeBerthDistanceMap[berth.id][ship.locAndDir.pos.x][ship.locAndDir.pos.y];
        profitBerths.push_back({berth.id, berth.totalValue + berth.futureValue / distance});
    }

    // 进行排序，如果收益为0，则比较estimateValue
    std::sort(profitBerths.begin(), profitBerths.end(), [&berths](std::pair<BerthID, float> &a, std::pair<BerthID, float> &b){
        // 收益都为0，比较estimateValue
        if (a.second == 0 && b.second == 0) return berths[a.first].estimateValue > berths[b.first].estimateValue;
        // 比较收益
        else return a.second > b.second;
    });

    // LOGI("排序完毕");

    BerthID assignedBerthId = -1;
    for (auto & profitBerth : profitBerths){
        if(canShipMoveToBerth(map, ship, berths[profitBerth.first])){
            assignedBerthId = profitBerth.first;
            break;
        }
    }

    // 泊位选取有误
    if (assignedBerthId == -1){
        LOGE("scheduleShipAtShipShops，分配泊位失败：", ship);
        return;
    }

    ship.updateMoveToBerthStatus(assignedBerthId, VectorPosition(berths[assignedBerthId].pos, berths[assignedBerthId].orientation));
}

// 计算船在该泊位上能得到多少收益（只考虑泊位上已有的货物）,返回{价值， 装货时间}
std::pair<int, int> GreedyShipScheduler::calculateShipProfitInBerth(Map &map, Ship &ship, Berth &berth){
    int startIndex = 0;
    int endIndex = 0;
    int value = 0;

    // 计算船留在原地的价值
    if (berth.id == ship.berthId) return {berth.totalValue - berth.residue_value + berth.futureValue, std::min(static_cast<int>(ship.nowCapacity() / berth.velocity), static_cast<int>(berth.reached_goods.size() / berth.velocity))};
    
    startIndex = berth.reached_goods.size() - berth.residue_num;    //船到该泊位上装载货物的起始id
    endIndex = std::min(startIndex + ship.nowCapacity(), static_cast<int>(berth.reached_goods.size()));   //船能装载的货物

    for( int i = startIndex; i < endIndex; i++)
        value += berth.reached_goods[i].value;
    
    return {value, endIndex - startIndex};
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
        // int timeToBerth = map.maritimeBerthDistanceMap[ship.berthId][targetBerthPos.x][targetBerthPos.y];
        int timeToBerth = map.berthToBerthDistance[ship.berthId][berth.id];
        timeCost += timeToBerth;
    } 

    if (CURRENT_FRAME + timeCost + 10<= 15000) return true;
    else return false;
}

// 当船从当前泊位移动到其他泊位时，更新泊位相关参数
void GreedyShipScheduler::updateBerthWhenShipMove(Ship &ship,std::vector<Berth> &berths,BerthID targetId){
    if(ship.berthId == -1) return;
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
    // #ifdef DEBUG
    // assert(berths[nowBerthId].shipInBerthNum >= 0);
    // assert(berths[targetId].shipInBerthNum >= 0);
    // #endif
}


// 获取最近的交货点
// todo 后续要判断该虚拟点是否能在结束前到达
int GreedyShipScheduler::allocateDelivery( Berth &berth){
    int deliveryId = berth.distsToDelivery[0].first;
    return deliveryId;
}


// 判断船是否可以前往一个泊位
// 时间足够去虚拟点 && 未达到泊位的船容量（如果泊位被预定但是 船还没到达，可以判断是否可以进去）
bool GreedyShipScheduler::canShipMoveToBerth(Map &map, Ship &ship, Berth &berth){
    if (ship.berthId == berth.id) return false;
    // 判断时间是否足够
    int timeCostToBerth =0;
    if (ship.isIdle()){
        timeCostToBerth = map.maritimeBerthDistanceMap[berth.id][ship.locAndDir.pos.x][ship.locAndDir.pos.y];
    }
    // 泊位前往泊位
    else if(ship.berthId != -1){
        timeCostToBerth = map.berthToBerthDistance[ship.berthId][berth.id];
    }
    // 虚拟点前往泊位
    else{
        timeCostToBerth = map.berthToDeliveryDistance[berth.id][ship.deliveryId];
        LOGI("虚拟点前往泊位:", timeCostToBerth);
    }
    // 时间：移动到泊位距离 + 目标泊位到虚拟点距离 + 装货时间
    int deliveryId = allocateDelivery(berth);
    int timeCost = timeCostToBerth +  map.berthToDeliveryDistance[berth.id][deliveryId]
     + std::min(static_cast<int>(ship.nowCapacity() / berth.velocity), static_cast<int>(berth.reached_goods.size() / berth.velocity)) + 15;

    if (CURRENT_FRAME + timeCost + 2 > 15000){
        LOGI("时间不够去另一个泊位", berth.id, "，消耗时间：", timeCost);
        LOGI("前往泊位距离：", timeCostToBerth);
        LOGI("装货时间：", std::min(static_cast<int>(ship.nowCapacity() / berth.velocity), static_cast<int>(berth.reached_goods.size() / berth.velocity)));
        LOGI("泊位前往交货点距离：", map.berthToDeliveryDistance[berth.id][deliveryId]);
        ship.info();
        return false;
    }
    else{
        LOGI("时间足够去另一个泊位：", ship);
        LOGI("路径代价消耗：", timeCost);
    }
    
    // 判断是否达到泊位的船容量
    if(berth.shipInBerthNum < MAX_SHIP_NUM) 
        return true;
    // 可以在预定船来临前进入泊位
    // todo 视效果可以注释
    else if (berth.onRouteTime > timeCostToBerth + 10 + static_cast<int>(berth.reached_goods.size())) {
        LOGI("泊位容量已满，插队");
        berth.info();
        LOGI("船插队时长：", timeCostToBerth);
        berth.onRouteTime = 0;
        return true;
    }
    else 
        return false;
}