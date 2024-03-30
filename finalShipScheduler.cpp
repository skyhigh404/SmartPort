#include "FinalShipScheduler.h"

FinalShipScheduler::FinalShipScheduler(const std::vector<int> &cluster)
    : berthCluster(std::make_shared<std::vector<int>>(cluster)){}

std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> FinalShipScheduler::scheduleShips(Map &map, std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods, std::vector<Robot> &robots, int currentFrame) {
    // 1. 选定终局泊位和候选泊位，分配船只
    if(!hasInit) init(ships, berths, goods);    

    std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> actions;
    ShipActionSpace::ShipAction action;
    for(auto &ship : ships){
        switch (ship.state)
        {
        case 0:
            // 船在途中不做调度
            break;
        default:
            // 船在虚拟点
            if(ship.berthId == -1){
                action = handleShipAtVirtualPoint(ship, berths);
            }
            // 船应该前往虚拟点
            else if(shouldDepartBerth(ship, berths)){
                action = ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH,-1);
            }
            // 船在候选泊位上
            else if(isShipAtBackupBerth(ship)){
                action = handleShipAtBackupBerth(ship, berths);
            }
            // 船在终局泊位上
            else if(isShipAtFinalBerth(ship)){
                action = handleShipAtFinalBerth(ship, berths);
            }
            break;
        }
        // 解决冲突情况
        actions.push_back(std::make_pair(ship.id,action));
    }
}

// 处理船在候选泊位的情况
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipAtBackupBerth(Ship& ship, std::vector<Berth> &berths){

}

// 处理船在终局泊位的情况
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipAtFinalBerth(Ship& ship, std::vector<Berth> &berths){

}

// 处理船在虚拟点的情况
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipAtVirtualPoint(Ship& ship, std::vector<Berth> &berths){

}

// 初始化
void FinalShipScheduler::init(std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods){
    // 初始化边界变量
    for(auto &ship : ships) maxCapacity = std::max(maxCapacity,ship.capacity);
    for(auto &berth : berths) minVelocity = std::min(minVelocity,berth.velocity),maxTime = std::max(maxTime, berth.time);
    maxLoadTime = maxCapacity / minVelocity;

    // 更新船和泊位状态
    updateBerthStatus(ships, berths, goods);

    // 初始化聚簇的泊位信息
    for(auto &index : *berthCluster){
        clusters[berthCluster->at(index)].push_back(berths[index]);
    }

    // 选取终局泊位和对应的候选泊位
    selectFinalAndBackupBerths(berths);

    // 给船分配终局泊位和候选泊位
    assignFinalBerthsToShips(ships, berths);

    hasInit = true;
}


// 初始化泊位的状态
void FinalShipScheduler::updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths,std::vector<Goods> & goods){
    // 遍历泊位，初始化泊位的正常货物量和未到达货物量，计算泊位价值
    for(auto &berth : berths){
        berth.totalValue = 0;
        berth.residue_num = 0;
        berth.shipInBerthNum = 0;
        berth.residue_num = berth.reached_goods.size();
    }
    //  遍历货物，更新泊位的溢出货物量和价值
    // todo 根据货物到达泊位的距离进行加权影响
    for(auto &good : goods){
        // todo 后续需要考虑泊位禁用情况
        if((good.status == 1 || good.status == 2) && berths[good.distsToBerths[0].first].isEnabled){
             berths[good.distsToBerths[0].first].residue_num += 1;
        }
    }
}


// 判断船只是否需要前往虚拟点
bool FinalShipScheduler::shouldDepartBerth(Ship &ship,std::vector<Berth> &berths){
    // 1. 船满了，前往虚拟点
    if (ship.now_capacity <= 0)  return true;
    // 2. 游戏快结束了，前往虚拟点
    // todo 15000改成全局变量；缓冲时间变成超参
    else if(ship.berthId != -1 && 15000 - CURRENT_FRAME <= berths[ship.berthId].time + 2) return true;
    else return false;
}

// 配对终局泊位和候选泊位
void FinalShipScheduler::selectFinalAndBackupBerths(std::vector<Berth> &berths){
    std::vector<Berth> berths_sort(berths);
    // 按照前往虚拟点时间降序排列
    std::sort(berths_sort.begin(), berths_sort.end(),[](Berth &a, Berth &b){
        return a.time < b.time;
    });

    // time最小的ShipNum个泊位为终局泊位，其余为候选泊位
    finalBerths = std::vector<Berth>(berths_sort.begin(), berths_sort.begin() + SHIPNUMS);
    backupBerths = std::vector<Berth>(berths_sort.begin() + SHIPNUMS, berths_sort.end());

    // 每一对终局泊位和候选泊位的residue_num之和尽量平衡
    std::sort(finalBerths.begin(), finalBerths.end(), [](Berth &a, Berth &b){
        return a.residue_num > b.residue_num;
    });
    std::sort(backupBerths.begin(), finalBerths.end(), [](Berth &a, Berth &b){
        return a.residue_num < b.residue_num;
    });

    // 初始化终局泊位和候选泊位变量
    for(int i = 0; i < finalBerths.size(); i++){
        finalToShip[finalBerths[i].id] = -1;
        finalToBackup[finalBerths[i].id] = backupBerths[i].id;
        backupToFinal[backupBerths[i].id] = finalBerths[i].id;
    }

    // 终局泊位按照去虚拟点的时间排序
    std::sort(finalBerths.begin(), finalBerths.end(), [](Berth &a, Berth &b){
        return a.time < b.time;
    });
}

// 为船选定终局泊位
void FinalShipScheduler::assignFinalBerthsToShips(std::vector<Ship> &ships, std::vector<Berth> &berths){
    // 初始化船和最终泊位的映射关系
    for(auto &ship : ships){
        shipToFinal[ship.id] = -1;
    }

    // 设置船分配泊位的优先级
    std::vector<Ship> ships_sort(ships);
    std::unordered_map<ShipID, int> shipPriorityMap;
    for(auto &ship : ships){
        // 船在前往候选泊位途中，优先级最高
        if(isShipOnRouteToBackBerth(ship)){
            shipPriorityMap[ship.id] = 0;
        }
        // 船在候选泊位上，优先级很高
        else if(isShipAtBackupBerth(ship)){
            shipPriorityMap[ship.id] = 1;
        }
        // 船在前往其他泊位途中(划分时间较长，所以安排去虚拟点时间较小的泊位)
        else if(ship.state == 0){
            shipPriorityMap[ship.id] = 2;
        }
        // 船在其他泊位上
        else{
            shipPriorityMap[ship.id] = 3;
        }
    }
    // 按照优先级排序
    std::sort(ships_sort.begin(), ships_sort.end(), [&shipPriorityMap](Ship &a, Ship &b){
        // 船在候选泊位上优先级最高
        if(shipPriorityMap[a.id] == shipPriorityMap[b.id]){
            switch (shipPriorityMap[a.id])
            {
            case 0:
                // 耗时长的优先
                return a.remainingTransportTime >= b.remainingTransportTime;
                break;
            case 1:
                // 容量高的优先
                return a.now_capacity >= b.now_capacity;
                break;
            case 2:
                // 耗时长的优先
                return a.remainingTransportTime >= b.remainingTransportTime;
                break;
            default:
                // 容量大的优先
                return a.now_capacity > b.now_capacity;
                break;
            }
        }else{
            return shipPriorityMap[a.id] < shipPriorityMap[b.id];
        }
    });

    // 分配终局泊位
    for(auto &ship : ships_sort){
        BerthID finalBerthId = getShipFinalBerth(ship);
        shipToFinal[ship.id] = finalBerthId;
        finalToShip[finalBerthId] = ship.id;
    }
}

// 获取船的终局泊位
BerthID FinalShipScheduler::getShipFinalBerth(Ship &ship){
    if(shipToFinal[ship.id] != -1) return shipToFinal[ship.id];
    else{
        if((isShipOnRouteToBackBerth(ship) || isShipAtBackupBerth(ship)) && !hasAssigned(ship.berthId)) {
            return ship.berthId;
        }
        // 遍历空闲泊位
        for(int i = 0;i < finalBerths.size(); i++){
            if(!hasAssigned(finalBerths[i].id)) return finalBerths[i].id;
        }
        #ifdef  DEBUG
        LOGE("给船分配终局泊位失败！");
        exit();
        #endif
        return -1;
    }
}

// 获取船的候选泊位
BerthID FinalShipScheduler::getShipBackupBerth(Ship &ship){

}

// 判断终局泊位是否已经分配
bool FinalShipScheduler::hasAssigned(BerthID berthId){
    if(finalToShip[berthId] != -1) return true;
    else return false;
}

// 判断船是否在指定的候选泊位或者终局泊位上
bool FinalShipScheduler::isShipAtAssignedBerth(Ship &ship){

}

// 判断船是否在候选泊位上
bool FinalShipScheduler::isShipAtBackupBerth(Ship &ship){

}

// 判断船是否在前往候选泊位途中
bool FinalShipScheduler::isShipOnRouteToBackBerth(Ship &ship){

}

// 判断船是否在终局泊位上
bool FinalShipScheduler::isShipAtFinalBerth(Ship &ship){

}

// 判断船是否有时间前往候选泊位
bool FinalShipScheduler::canReachBackupBerth(Ship &ship){

}

// 判断船是否有时间前往终局泊位
bool FinalShipScheduler::canReachFinalBerth(Ship &ship){

}

// 判断船是否能去虚拟点后再回来指定泊位，最后再前往虚拟点
bool FinalShipScheduler::canDepartBerth(Ship &ship, Berth &berth){

}

// 判断船是否必须前往虚拟点
bool FinalShipScheduler::shouldDepartBerth(Ship &ship,std::vector<Berth> &berths){

}

// 禁用泊位
void FinalShipScheduler::disableBerth(Berth &berth){

}


// 初始化泊位的状态
void FinalShipScheduler::updateBerthStatus(std::vector<Ship> &ships,std::vector<Berth> &berths,std::vector<Goods> & goods){

}