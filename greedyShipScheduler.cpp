#include "greedyShipScheduler.h"


std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> GreedyShipScheduler::scheduleShips(Map &map, std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods, std::vector<Robot> &robots, int currentFrame) {
    // 1. 更新泊位和货物的状态
    updateBerthStatus(ships,berths);

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
}

// 处理船在泊位上的情况
ShipActionSpace::ShipAction
GreedyShipScheduler::handleShipAtBerth(Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 前往虚拟
    if(shouldDepartBerth(ship, berths)){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH,-1);
    }
    // 有货转货
    else if(isThereGoodsToLoad(berths[ship.berthId])){
        BerthID berthId = ship.berthId;
        int shipment = std::min(static_cast<int>(berths[berthId].reached_goods.size()),berths[berthId].velocity);
        int res = ship.loadGoods(shipment); // 装货
        berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);
    }
    // 分配泊位
    else{
        BerthID tagetId = findBestBerthForShip(ship, berths, goods);
        if(tagetId != ship.berthId && tagetId != -1) return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, tagetId);
        else return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, tagetId);
    }

}

// 处理船在虚拟点的情况
ShipActionSpace::ShipAction
handleShipInEnd(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 寻找当前最优的泊位
}

// 处理在泊位外等待的情况
ShipActionSpace::ShipAction
GreedyShipScheduler::handleShipWaiting(const Ship &ship,std::vector<Berth> &berths,std::vector<Goods> &goods){
    // 寻找当前最优的泊位
}


// 初始化泊位的状态
void GreedyShipScheduler::updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths){
    // 计算泊位的溢出货物量
    // 计算当前泊位上的船只数量
    // 计算当前泊位的货物价值
}

// 计算一段时间内泊位的价值收益
float GreedyShipScheduler::calculateBerthFutureValue(std::vector<Berth> &berths, std::vector<Goods> &goods,int timeSpan){
    // 计算当前溢出货物和未来timeSpan时间内可能到达货物的价值之和
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
    if(berth.reached_goods.size() == 0) return true;
    else return false;
}

// 为船找到最佳泊位，返回泊位id（局部最优）
// 考虑：泊位前往虚拟点的时间代价、泊位未来的收益、泊位货物量和船容量的匹配
BerthID GreedyShipScheduler::findBestBerthForShip(const Ship &ship, const std::vector<Berth> &berths, const std::vector<Goods> &goods){
    // 泊位进行收益排序（根据未来一段时间内的收益）
    // 找到容量匹配而且收益更高的泊位
    // 返回泊位id
}

// 判断船可以前往其他泊位
bool GreedyShipScheduler::canMoveBerth(const Ship &ship,const Berth &berth){
    // 如果船前往其他泊位后还有时间前往虚拟点，则返回true
    // todo 泊位移动距离和缓冲时间改成超参
    if (CURRENT_FRAME + 500 + berth.time + static_cast<int>(ship.now_capacity / berth.velocity) <= 15000) return true;
    else return false;
}