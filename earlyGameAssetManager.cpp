#include "earlyGameAssetManager.h"


void EarlyGameAssetManager::setParameter(const Params &params)
{
    maxRobotNum = params.maxRobotNum;
    maxShipNum = params.maxShipNum;
    robotPurchaseAssign = params.robotPurchaseAssign;
    shipPurchaseAssign = params.shipPurchaseAssign;
    startNum = params.startNum;
}

void EarlyGameAssetManager::init(const Map& map, const std::vector<Berth> &berths)
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
    // robotPurchaseAssign = std::vector<int>(landseaBlocks.size(), 0);
    // shipPurchaseAssign = std::vector<int>(landseaBlocks.size(), 0);
    // divideSeaConnectedBlocks(berths, gameMap.deliveryLocations, gameMap);
}

std::vector<PurchaseDecision> EarlyGameAssetManager::makePurchaseDecision(const Map &gameMap,
                                                    const std::vector<Goods> &goods,
                                                    const std::vector<Robot> &robots,
                                                    const std::vector<Ship> &ships,
                                                    const std::vector<Berth> &berths,
                                                    int currentFunds,
                                                    int currentTime)
{
    PurchaseDecision robotDecision, shipDecision;
    // 判断要不要购买机器人/轮船
    if (needToBuyRobot(robots, goods, gameMap, currentFunds)) {
        Point2d shopPos = buyRobot(robots, goods, gameMap, currentFunds);
        robotDecision = PurchaseDecision{AssetType::ROBOT, shopPos, 1};
    }
    if (needToBuyShip(ships, goods, gameMap, currentFunds)) {
        Point2d shopPos = buyShip(ships, goods, gameMap, currentFunds);
        shipDecision = PurchaseDecision{AssetType::SHIP, shopPos, 1};
    }
    return {robotDecision, shipDecision};
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

void EarlyGameAssetManager::divideLandAndSeaConnectedBlocks(const std::vector<Berth> &berths, const Map &map)
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
            const Berth& berth = berths[j];
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
        landseaBlocks[i].berths = connectedBerths;
        landseaBlocks[i].robotShops = availableRobotShops;
        landseaBlocks[i].shipShops = availableShipShops;
    }
}
bool EarlyGameAssetManager::needToBuyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    // 超出最大限制
    if (robots.size() >= maxRobotNum) return false;
    if (currentFunds < robotPrice) return false;
    return true;
}
bool EarlyGameAssetManager::needToBuyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    // 超出最大限制
    if (ships.size() >= maxShipNum) return false;
    if (currentFunds < shipPrice) return false;
    return true;
}

Point2d EarlyGameAssetManager::buyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    for (int phase=0; phase<robotPurchaseAssign.size(); phase++) {
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
    for (int phase=0; phase<shipPurchaseAssign.size(); phase++) {
        // 按阶段进行购买
        for (int i=0; i<landseaBlocks.size(); i++) {
            // 当前联通块已完成本阶段购买
            if (purchasedShipNum[i] >= shipPurchaseAssign[i][phase]) continue;
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