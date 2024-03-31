#include "FinalShipScheduler.h"

//  设置参数
void FinalShipScheduler::setParameter(const Params &params)
{
    ABLE_DEPART_SCALE = params.ABLE_DEPART_SCALE;
    MAX_SHIP_NUM = params.MAX_SHIP_NUM;
    TIME_TO_WAIT = params.TIME_TO_WAIT;
    CAPACITY_GAP = params.CAPACITY_GAP;
}

FinalShipScheduler::FinalShipScheduler(const std::vector<int> &berthCluster, const std::vector<std::vector<Berth>> &clusters)
    : berthCluster(std::make_shared<std::vector<int>>(berthCluster)),clusters(std::make_shared<std::vector<std::vector<Berth>>>(clusters)){}

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
                action = ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH,ship.berthId);
            }
            // 船在候选泊位上
            else if(isShipAtBackupBerth(ship)){
                action = handleShipAtBackupBerth(ship, berths);
            }
            // 船在终局泊位上
            else {
                #ifdef DEBUG
                assert(isShipAtFinalBerth(ship));
                LOGE("终局船调度分支选择出错");
                #endif
                action = handleShipAtFinalBerth(ship, berths);
            }
            break;
        }
        // 解决冲突情况
        actions.push_back(std::make_pair(ship.id,action));
    }
    return actions;
}

// 处理船在候选泊位的情况
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipAtBackupBerth(Ship& ship, std::vector<Berth> &berths){
    BerthID backupBerthId = getShipBackupBerth(ship);
    BerthID finalBerthId = getShipFinalBerth(ship);
    // 不在选定的候选泊位上
    if(!isShipAtAssignedBerth(ship)){
        return handleShipNotAtAssignedBerth(ship, berths);
    }
    // 还有时间去放货
    // todo 超参数
    if(shouldDepartAndReturn(ship, berths[finalBerthId], berths) && ship.capacityScale() < 0.8){
        // 禁用候选泊位
        berths[backupBerthId].disable();
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH, ship.berthId);
    }
    //  必须前往最终泊位
    else if (shouldReachFinalBerth(ship, berths)){
        // 禁用候选泊位
        berths[backupBerthId].disable();
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, finalBerthId); 
    }
    // 有货装货
    else if(isThereGoodsToLoad(berths[backupBerthId])){
        loadGoodAtBerth(ship, berths);
    }
    return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, ship.berthId); 
    
}

// 处理船在终局泊位的情况
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipAtFinalBerth(Ship& ship, std::vector<Berth> &berths){
    BerthID backupBerthId = getShipBackupBerth(ship);
    BerthID finalBerthId = getShipFinalBerth(ship);
    #ifdef DEBUG
    assert(ship.berthId == finalBerthId);
    #endif
    // 不在选定的候选泊位上
    if(!isShipAtAssignedBerth(ship)){
       return handleShipNotAtAssignedBerth(ship, berths);
    }
    //  判断能否前往候选泊位
    if (canReachBackupBerth(ship, berths)){
         return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, backupBerthId); 
    }
    // 禁用候选泊位
    berths[backupBerthId].disable();
    // 容量不多，判断能否前往虚拟点
    if (ship.capacityScale() < ABLE_DEPART_SCALE && shouldDepartAndReturn(ship, berths[finalBerthId], berths)){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH, ship.berthId);
    }
    //  有货装货
    else if(isThereGoodsToLoad(berths[finalBerthId])){
        loadGoodAtBerth(ship, berths);
    }
    return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, ship.berthId); 
}

//  处理船不在指定泊位的情况
// 处理船不在指定泊位情况（只有开局会出现）
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipNotAtAssignedBerth(Ship& ship, std::vector<Berth> &berths){
    BerthID backupBerthId = getShipBackupBerth(ship);
    BerthID finalBerthId = getShipFinalBerth(ship);
    // 判断船的容量是否需要去虚拟点
    if(ship.capacityScale() < ABLE_DEPART_SCALE){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::DEPART_BERTH, ship.berthId);
    }
    // 判断船是否能前往候选泊位
    else if(canReachBackupBerth(ship, berths)){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, backupBerthId); 
    }
    // 否则判断船是否能前往最终泊位
    else if(canReachFinalBerth(ship, berths)){
        // 禁用候选泊位
        berths[backupBerthId].disable();
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, finalBerthId); 
    }
    // 有货装货
    else if(isThereGoodsToLoad(berths[ship.berthId])){
        loadGoodAtBerth(ship, berths);
    }
    return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::CONTINUE, ship.berthId);
}

// 处理船在虚拟点的情况
ShipActionSpace::ShipAction
FinalShipScheduler::handleShipAtVirtualPoint(Ship& ship, std::vector<Berth> &berths){
    BerthID backupBerthId = getShipBackupBerth(ship);
    BerthID finalBerthId = getShipFinalBerth(ship);
    // 判断能否前往候选泊位
    if(canReachBackupBerth(ship, berths)){
        return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, backupBerthId); 
    }
    //  前往最终泊位
    return ShipActionSpace::ShipAction(ShipActionSpace::ShipActionType::MOVE_TO_BERTH, finalBerthId); 
}

// 初始化
void FinalShipScheduler::init(std::vector<Ship> &ships, std::vector<Berth> &berths, std::vector<Goods> &goods){
    // 初始化边界变量
    for(auto &ship : ships) maxCapacity = std::max(maxCapacity,ship.capacity);
    for(auto &berth : berths) minVelocity = std::min(minVelocity,berth.velocity),maxTime = std::max(maxTime, berth.time);
    maxLoadTime = maxCapacity / minVelocity;

    // 更新船和泊位状态
    updateBerthStatus(ships, berths, goods);

    // // 初始化聚簇的泊位信息
    // for(auto &index : *berthCluster){
    //     clusters[berthCluster->at(index)].push_back(berths[index]);
    // }
    
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
    for(auto &good : goods){
        if((good.status == 1 || good.status == 2) && berths[good.distsToBerths[0].first].isEnable()){
             berths[good.distsToBerths[0].first].residue_num += 1;
        }
    }
}


// // 判断船只是否需要前往虚拟点
// bool FinalShipScheduler::shouldDepartBerth(Ship &ship,std::vector<Berth> &berths){
//     // 1. 船满了，前往虚拟点
//     if (ship.now_capacity <= 0)  return true;
//     // 2. 游戏快结束了，前往虚拟点
//     // todo 15000改成全局变量；缓冲时间变成超参
//     else if(ship.berthId != -1 && 15000 - CURRENT_FRAME <= berths[ship.berthId].time + 2) return true;
//     else return false;
// }

// 配对终局泊位和候选泊位
void FinalShipScheduler::selectFinalAndBackupBerths(std::vector<Berth> &berths){
    // 每个聚簇类按照前往虚拟点时间升序排列
    for(int index = 0;index < clusters->size(); index++){
        std::vector<Berth> cluster_sort(clusters->at(index));
        std::sort(cluster_sort.begin(), cluster_sort.end(),[](Berth &a, Berth &b){
            return a.time < b.time;
        });
        // 选取第一个作为终局泊位
        finalBerths.push_back(cluster_sort[0]);
        for(int i = 1;i < cluster_sort.size(); i++){
            backupBerths.push_back(cluster_sort[i]);
        }
    }

    // std::vector<Berth> berths_sort(berths);
    // // 按照前往虚拟点时间降序排列
    // std::sort(berths_sort.begin(), berths_sort.end(),[](Berth &a, Berth &b){
    //     return a.time < b.time;
    // });

    // // time最小的ShipNum个泊位为终局泊位，其余为候选泊位
    // finalBerths = std::vector<Berth>(berths_sort.begin(), berths_sort.begin() + SHIPNUMS);
    // backupBerths = std::vector<Berth>(berths_sort.begin() + SHIPNUMS, berths_sort.end());

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
        BerthID finalBerthId = assignFinalBerth(ship);
        shipToFinal[ship.id] = finalBerthId;
        finalToShip[finalBerthId] = ship.id;
    }
}

// 为单个船分配终局泊位
BerthID FinalShipScheduler::assignFinalBerth(Ship &ship){
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

// 获取船的终局泊位
BerthID FinalShipScheduler::getShipFinalBerth(Ship &ship){
    #ifdef DEBUG
    if(shipToFinal[ship.id] == -1){
        LOGE("船只分配终局泊位出错，-1");
    }
    #endif
    return shipToFinal[ship.id];
}

// 获取船的候选泊位
BerthID FinalShipScheduler::getShipBackupBerth(Ship &ship){
    // 如果船还没有被分配终局泊位，报错
    if(shipToFinal[ship.id] == -1){
        #ifdef  DEBUG
        LOGE("获取船的候选泊位失败！");
        exit();
        #endif
        return -1;
    }
    //  如果终局泊位还没有被分配候选泊位，报错
    if(finalToBackup[shipToFinal[ship.id]]){
        #ifdef  DEBUG
        LOGE("获取终局泊位的候选泊位失败！");
        exit();
        #endif
        return -1;
    }
    return finalToBackup[shipToFinal[ship.id]];
}

// 判断终局泊位是否已经分配
bool FinalShipScheduler::hasAssigned(BerthID berthId){
    if(finalToShip[berthId] != -1) return true;
    else return false;
}

// 判断船是否在指定的候选泊位或者终局泊位上
bool FinalShipScheduler::isShipAtAssignedBerth(Ship &ship){
    if(ship.state == 0) return false;
    return ship.berthId == getShipBackupBerth(ship) || ship.berthId == getShipFinalBerth(ship);
}

// 判断船是否在候选泊位上
bool FinalShipScheduler::isShipAtBackupBerth(Ship &ship){
    return backupToFinal.count(ship.berthId) != 0 && ship.state != 0;
}

// 判断船是否在前往候选泊位途中
bool FinalShipScheduler::isShipOnRouteToBackBerth(Ship &ship){
    return ship.berthId == getShipBackupBerth(ship) && ship.state == 0;
}

// 判断船是否在终局泊位上
bool FinalShipScheduler::isShipAtFinalBerth(Ship &ship){
    return finalToBackup.count(ship.berthId) != 0 && ship.state != 0;
}

// 判断船是否有时间前往候选泊位
bool FinalShipScheduler::canReachBackupBerth(Ship &ship, std::vector<Berth> &berths){
    BerthID backupBerthId = getShipBackupBerth(ship);
    BerthID finalBerthId = getShipFinalBerth(ship);
    int timeCost = berths[finalBerthId].time + 500 + maxLoadTime;
    if(ship.state != 0){
        timeCost += 500;
    }else{
        timeCost += ship.remainingTransportTime;
    }
    //  todo 超参
    if(timeCost + CURRENT_FRAME <= 15000) return true;
    else return false;
}

// 判断船是否有时间前往终局泊位
bool FinalShipScheduler::canReachFinalBerth(Ship &ship, std::vector<Berth> &berths){
    BerthID finalBerthId = getShipFinalBerth(ship);
    int timeCost = berths[finalBerthId].time + maxLoadTime;
    if(ship.state != 0){
        timeCost += 500;
    }else{
        timeCost += ship.remainingTransportTime;
    }
    //  todo 超参
    if(timeCost + CURRENT_FRAME <= 15000) return true;
    else return false;
}

// 判断船是否应该前往终局泊位
bool FinalShipScheduler::shouldReachFinalBerth(Ship &ship, std::vector<Berth> &berths){
    BerthID finalBerthId = getShipFinalBerth(ship);
    int timeCost = berths[finalBerthId].time + maxLoadTime;
    if(ship.state != 0){
        timeCost += 500;
    }else{
        timeCost += ship.remainingTransportTime;
    }
    //  todo 超参，缓冲帧
    if(std::clamp(15000 - 2,timeCost + CURRENT_FRAME, 15000)) return true;
    else return false;
}

// 判断船是否能去虚拟点后再回来指定泊位，最后再前往虚拟点
bool FinalShipScheduler::canDepartAndReturn(Ship &ship, Berth &berth, std::vector<Berth> &berths){
    //  todo 超参，可能并不需要maxTime个时间
    int timeCost = berth.time * 2 + maxLoadTime + berths[ship.berthId].time;
    if(timeCost + CURRENT_FRAME < 15000) return true;
    else return false;
}

// 判断船是否必须去虚拟点后再回来指定泊位，最后再前往虚拟点
bool FinalShipScheduler::shouldDepartAndReturn(Ship &ship, Berth &berth, std::vector<Berth> &berths){
    int timeCost = berth.time * 2 + maxLoadTime + berths[ship.berthId].time;
    //  todo 超参，缓冲帧
    if(std::clamp(15000 - 2,timeCost + CURRENT_FRAME, 15000)) return true;
    else return false;
}


// 判断船是否必须前往虚拟点
bool FinalShipScheduler::shouldDepartBerth(Ship &ship,std::vector<Berth> &berths){
    if (ship.now_capacity <= 0)  return true;
    if (ship.state == 0 || ship.berthId == -1) return false;
    int timeCost = berths[ship.berthId].time;
    if(std::clamp(15000 - 2,timeCost + CURRENT_FRAME, 15000)) return true;
    else return false;
}

// // 禁用泊位
// void FinalShipScheduler::disableBerth(Berth &berth){
//     #ifdef  DEBUG
//     if(berth.isEnabled == false) LOGE("重复禁用泊位");
//     #endif;

//     berth.isEnabled = false;
// }

// 判断泊位上是否有货物可装载
bool FinalShipScheduler::isThereGoodsToLoad(Berth &berth){
    if(berth.reached_goods.size() != 0) return true;
    else return false;
}

// 处理装货的状态
void FinalShipScheduler::loadGoodAtBerth(Ship &ship, std::vector<Berth> &berths){
    BerthID berthId = ship.berthId;
    int shipment = std::min(static_cast<int>(berths[berthId].reached_goods.size()),berths[berthId].velocity);
    int res = ship.loadGoods(shipment); // 装货
    berths[berthId].reached_goods.erase(berths[berthId].reached_goods.begin(),berths[berthId].reached_goods.begin() + res);
}