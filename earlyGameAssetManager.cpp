#include "earlyGameAssetManager.h"


void EarlyGameAssetManager::setParameter(const Params &params)
{
    maxRobotNum = params.maxRobotNum;
    maxShipNum = params.maxShipNum;
    robotPurchaseAssign = params.robotPurchaseAssign;
    timeToBuyShip = params.timeToBuyShip;
    shipPurchaseAssign = params.shipPurchaseAssign;
    startNum = params.startNum;
    landDistanceWeight = params.landDistanceWeight;
    deliveryDistanceWeight = params.deliveryDistanceWeight;
}

void EarlyGameAssetManager::init(const Map& map, std::vector<Berth> &berths)
{
    // 创建机器人、轮船商店
    for (int i=0;i<map.rows;i++) {
        for (int j=0;j<map.cols;j++) {
            // 机器人商店
            if (map.readOnlyGrid[i][j]==MapItemSpace::MapItem::ROBOT_SHOP) {
                robotShops.emplace_back(i,j);
            }
            // 轮船商店
            if (map.readOnlyGrid[i][j]==MapItemSpace::MapItem::SHIP_SHOP) {
                shipShops.emplace_back(i,j);
            }
        }
    }

    divideLandAndSeaConnectedBlocks(berths, map);
    
    purchasedRobotNum = std::vector<int>(landseaBlocks.size(), 0);
    purchasedShipNum = std::vector<int>(landseaBlocks.size(), 0);

    calBerthsEstimateValue(berths, map);
}

std::vector<PurchaseDecision> EarlyGameAssetManager::makePurchaseDecision(const Map &gameMap,
                                                    const std::vector<Goods> &goods,
                                                    const std::vector<Robot> &robots,
                                                    const std::vector<Ship> &ships,
                                                    const std::vector<Berth> &berths,
                                                    int currentFunds,
                                                    int currentTime)
{
    std::vector<PurchaseDecision> purchaseDecisions;
    // 判断要不要购买机器人/轮船
    if (needToBuyShip(ships, goods, gameMap, currentFunds, currentTime)) {
        Point2d shopPos = buyShip(ships, goods, gameMap, currentFunds);
        LOGI("购买轮船，当前资金：", currentFunds, "，购买点：", shopPos);
        if (shopPos != Point2d(-1,-1)) {
            currentFunds -= shipPrice;
            int assignId = getAssignId(shopPos, berths);
            purchaseDecisions.push_back(PurchaseDecision{AssetType::SHIP, shopPos, 1, assignId});
        }
    }
    if (needToBuyRobot(robots, goods, gameMap, currentFunds)) {
        Point2d shopPos = buyRobot(robots, goods, gameMap, currentFunds);
        LOGI("购买机器人，当前资金：", currentFunds, "，购买点：", shopPos);
        if (shopPos != Point2d(-1,-1)) {
            currentFunds -= robotPrice;
            int assignId = getAssignId(shopPos, berths);
            LOGI("购买机器人并分配泊位id:", assignId);
            purchaseDecisions.push_back(PurchaseDecision{AssetType::ROBOT, shopPos, 1, assignId});
        }
    }
    return purchaseDecisions;
}

int EarlyGameAssetManager::getAssignId(Point2d shopPos, const std::vector<Berth> &berths)
{
    return -1;
    // 游戏初期集中分配
    LandSeaBlock& lsb = landseaBlocks[0];

    // 不再集中分配（已度过第一阶段）
    if (purchasedRobotNum[0] > robotPurchaseAssign[0][0]) {
        // LOGI("停止集中搬货，已购机器人数目：", purchasedRobotNum[0], "，第一阶段机器人数目：", robotPurchaseAssign[0][0]);
        return -1;
    }

    // 如果不在第一大的连通块上则不分配具体泊位
    bool belongToLSB0 = false;
    for (auto& pos : lsb.robotShops) 
        if (pos == shopPos) {belongToLSB0 = true; break;}
    if (!belongToLSB0) {return -1;}

    int assignId = -1;
    float maxValue = 0;
    for (int i=0;i<lsb.berths.size();i++) {
        const Berth& berth = berths[lsb.berths[i].id];
        // LOGI(berth.id, ' ', berth.estimateValue);
        if (berth.estimateValue > maxValue) {
            assignId = berth.id;
            maxValue = berth.estimateValue;
        }
    }
    return assignId;
}

void EarlyGameAssetManager::calBerthsEstimateValue(std::vector<Berth>& berths, const Map& map)
{
    for (auto& berth : berths) {
        int totalLandDistance = 0, totalLandNum = 0;
        for (int i=0;i<map.rows;i++) 
            for (int j=0;j<map.cols;j++) 
                if (map.berthDistanceMap.at(berth.id)[i][j] < INT_MAX) {
                    totalLandDistance += map.berthDistanceMap.at(berth.id)[i][j];
                    totalLandNum++;
                }
        LOGI("totalLandDistance ",totalLandDistance, ", totalLandNum ", totalLandNum);
        berth.estimateValue += landDistanceWeight * 1.0 / (totalLandDistance * 1.0 / totalLandNum);

        int totalSeaDistance = 0, totalSeaNum = 0;
        for (auto& delivery : map.deliveryLocations)
            if (map.maritimeBerthDistanceMap.at(berth.id)[delivery.x][delivery.y] < INT_MAX) {
                totalSeaDistance += map.maritimeBerthDistanceMap.at(berth.id)[delivery.x][delivery.y];
                totalSeaNum++;
            }
        LOGI("totalSeaDistance ",totalSeaDistance, ", totalSeaNum ", totalSeaNum);
        berth.estimateValue += deliveryDistanceWeight * 1.0 / (totalSeaDistance * 1.0 / totalSeaNum);
    }
    LOGI("calBerthsEstimateValue:");
    for (auto& berth : berths) LOGI(berth.id, ' ', berth.estimateValue);
}

void EarlyGameAssetManager::divideLandConnectedBlocks(const std::vector<Berth> &berths, const Map &map)
{
    // 划分陆地连通块
    std::vector<bool> clustered(berths.size(), false);
    for (int i = 0; i < berths.size(); i++)
    {
        const Berth &berth = berths[i];
        if (!clustered[i])
        {
            std::vector<Berth> anotherClass;
            anotherClass.push_back(berths[i]);
            for (int j = i + 1; j < berths.size(); j++)
            {
                if (map.berthDistanceMap.at(i)[berths[j].pos.x][berths[j].pos.y] != INT_MAX)
                {
                    anotherClass.push_back(berths[j]);
                    clustered[j] = true;
                }
            }
            landBlocks.push_back(LandBlock{0, anotherClass});
        }
    }

    // 确定连通块大小
    for (LandBlock& lb: landBlocks) {
        int count_passableBlock = 0;
        for (int i=0;i<map.rows;i++) {
            for (int j=0;j<map.cols;j++) {
                if (map.berthDistanceMap.at(lb.berths[0].id)[i][j] < INT_MAX) 
                    count_passableBlock++;
            }
        }
        lb.size = count_passableBlock;
    }
}

void EarlyGameAssetManager::divideSeaConnectedBlocks(const std::vector<Berth> &berths, const std::vector<Point2d> &deliveryLocations, const Map &map)
{
    // 划分海洋连通块
    std::vector<bool> clustered(deliveryLocations.size(), false);
    for (int i = 0; i < deliveryLocations.size(); i++)
    {
        const Point2d& delivery1 = deliveryLocations[i];
        if (!clustered[i])
        {
            std::vector<Point2d> anotherClass;
            anotherClass.push_back(deliveryLocations[i]);
            for (int j = i + 1; j < deliveryLocations.size(); j++)
            {
                const Point2d& delivery2 = deliveryLocations[j];
                for (int k = 0; k < berths.size(); k++)
                    if (map.maritimeBerthDistanceMap.at(k)[delivery1.x][delivery1.y] != INT_MAX && map.maritimeBerthDistanceMap.at(k)[delivery2.x][delivery2.y] != INT_MAX)
                    {
                        anotherClass.push_back(deliveryLocations[j]);
                        clustered[j] = true;
                    }
            }
            seaBlocks.push_back(SeaBlock{0, anotherClass});
        }
    }

    // 确定连通块大小
    // for (SeaBlock& sb: seaBlocks) {
    //     int count_passableBlock = 0;
    //     for (int i=0;i<map.rows;i++) {
    //         for (int j=0;j<map.cols;j++) {
    //             if (map.maritimeBerthDistanceMap.at(sb.berths[0].id)[i][j] < INT_MAX) 
    //                 count_passableBlock++;
    //         }
    //     }
    //     sb.size = count_passableBlock;
    // }
}

void EarlyGameAssetManager::divideLandAndSeaConnectedBlocks(std::vector<Berth> &berths, const Map &map)
{
    const std::vector<Point2d> &deliveryLocations = map.deliveryLocations;
    std::vector<bool> clustered(deliveryLocations.size(), false);
    // 对交货点进行连通性聚类
    for (int i = 0; i < deliveryLocations.size(); i++)
    {
        const Point2d& delivery1 = deliveryLocations[i];
        if (!clustered[i])
        {
            std::vector<Point2d> anotherDeliveryLocations;
            anotherDeliveryLocations.push_back(deliveryLocations[i]);
            for (int j = i + 1; j < deliveryLocations.size(); j++)
            {
                const Point2d& delivery2 = deliveryLocations[j];
                for (int k = 0; k < berths.size(); k++)
                    if (map.maritimeBerthDistanceMap.at(k)[delivery1.x][delivery1.y] != INT_MAX && map.maritimeBerthDistanceMap.at(k)[delivery2.x][delivery2.y] != INT_MAX)
                    {
                        anotherDeliveryLocations.push_back(deliveryLocations[j]);
                        clustered[j] = true;
                    }
            }
            landseaBlocks.push_back(LandSeaBlock{0, {}, anotherDeliveryLocations, {}, {}});
        }
    }

    for (int i=0;i<landseaBlocks.size();i++) {
        LandSeaBlock& lsb = landseaBlocks[i];
        // 找出相连通的泊位
        std::vector<Berth> connectedBerths;
        for (int j=0;j<berths.size();j++) {
            Berth& berth = berths[j];
            if (map.maritimeBerthDistanceMap.at(berth.id)[lsb.deliveryLocations[0].x][lsb.deliveryLocations[0].y] != INT_MAX) {
                connectedBerths.push_back(berth);
            }
        }
        // 找出可用的机器人购买点
        std::vector<Point2d> availableRobotShops;
        for (int j=0;j<robotShops.size();j++) {
            Point2d& rs = robotShops[j];
            for (int k=0;k<connectedBerths.size();k++) {
                const Berth& berth = connectedBerths[k];
                if (map.berthDistanceMap.at(berth.id)[rs.x][rs.y] != INT_MAX) {
                    availableRobotShops.push_back(robotShops[j]);
                    break;
                }
            }
        }
        // 找出可用的轮船购买点
        std::vector<Point2d> availableShipShops;
        for (int j=0;j<shipShops.size();j++) {
            Point2d& ss = shipShops[j];
            for (int k=0;k<connectedBerths.size();k++) {
                const Berth& berth = connectedBerths[k];
                if (map.maritimeBerthDistanceMap.at(berth.id)[ss.x][ss.y] != INT_MAX) {
                    availableShipShops.push_back(shipShops[j]);
                    break;
                }
            }
        }
        // 找出连通块的陆地面积
        int landSize = 0;
        for (int j=0;j<map.rows;j++) {
            for (int k=0;k<map.cols;k++) {
                if (map.berthDistanceMap.at(connectedBerths[0].id)[j][k] < INT_MAX) {
                    landSize++;
                }
            }
        }
        landseaBlocks[i].landSize = landSize;
        landseaBlocks[i].berths = connectedBerths;
        landseaBlocks[i].robotShops = availableRobotShops;
        landseaBlocks[i].shipShops = availableShipShops;
    }

    std::sort(landseaBlocks.begin(), landseaBlocks.end(), [&](const LandSeaBlock& lhs, const LandSeaBlock& rhs) {
        return lhs.landSize > rhs.landSize; // 根据陆地面积进行降序排序
    });
}

bool EarlyGameAssetManager::needToBuyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    // 超出最大限制
    if (robots.size() >= maxRobotNum) return false;
    if (currentFunds < robotPrice) return false;
    return true;
}
bool EarlyGameAssetManager::needToBuyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds, int currentTime)
{
    if (purchasedShipNum[0]==0) return true;
    // 超出最大限制
    if (ships.size() >= maxShipNum) return false;
    if (currentFunds < shipPrice) return false;
    if (currentTime < timeToBuyShip) return false;
    return true;
}

Point2d EarlyGameAssetManager::buyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    for (int phase=0; phase<robotPurchaseAssign[0].size(); phase++) {
        // 按阶段进行购买
        for (int i=0; i<landseaBlocks.size(); i++) {
            // 当前联通块已完成本阶段购买
            if (purchasedRobotNum[i] >= robotPurchaseAssign[i][phase]) continue;
            // 为当前联通块购买机器人：找合适的购买点
            purchasedRobotNum[i]++; // 暂时在这更新，可能要移动到processFramedata去
            return getProperRobotShop(landseaBlocks[i], gameMap);
        }
    }
    // 不购买 或 购买失败
    return Point2d(-1,-1);
}

Point2d EarlyGameAssetManager::buyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    for (int phase=0; phase<shipPurchaseAssign[0].size(); phase++) {
        // 按阶段进行购买
        for (int i=0; i<landseaBlocks.size(); i++) {
            // 当前联通块已完成本阶段购买
            if (purchasedShipNum[i] >= shipPurchaseAssign[i][phase]) continue;
            // 加入timeSchudeler
            // 为当前联通块购买机器人：找合适的购买点
            purchasedShipNum[i]++; // 暂时在这更新，可能要移动到processFramedata去
            return getProperShipShop(landseaBlocks[i], gameMap);
        }
    }
    // 不购买 或 购买失败
    return Point2d(-1,-1);
}

Point2d EarlyGameAssetManager::getProperRobotShop(LandSeaBlock& block, const Map &gameMap)
{
    if (!block.robotShops.empty())
    return block.robotShops[0];
    //未完成
    return Point2d(-1,-1);
}
Point2d EarlyGameAssetManager::getProperShipShop(LandSeaBlock& block, const Map &gameMap)
{
    if (!block.shipShops.empty())
        return block.shipShops[0];
    //未完成
    return Point2d(-1,-1);
}