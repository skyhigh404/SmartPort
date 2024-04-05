#include "earlyGameAssetManager.h"


void EarlyGameAssetManager::setParameter(const Params &params)
{

}

void EarlyGameAssetManager::init(const Map& map, const std::vector<Berth> &berths, const Map &gameMap)
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

    divideLandConnectedBlocks(berths, gameMap);
    divideSeaConnectedBlocks();
}

std::vector<PurchaseDecision> EarlyGameAssetManager::makePurchaseDecision(const Map &gameMap,
                                                    const std::vector<Goods> &goods,
                                                    const std::vector<Robot> &robots,
                                                    const std::vector<Ship> &ships,
                                                    const std::vector<Berth> &berths,
                                                    int currentFunds,
                                                    int currentTime)
{
    // 判断要不要购买机器人/轮船
    if (needToBuyRobot(robots, goods, gameMap, currentFunds)) {
        buyRobot(robots, goods, gameMap, currentFunds);
    }
    if (needToBuyShip(ships, goods, gameMap, currentFunds)) {}
    return {};
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

void EarlyGameAssetManager::buyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    // 评估购买点:根据连通块大小排序然后优先选择最大的购买
    std::vector<int> shopValue(robotShops.size(), 0);
    for (int i=0;i<robotShops.size();i++) {

    }
    // 购买
}
bool EarlyGameAssetManager::buyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds)
{
    return false;
}

void EarlyGameAssetManager::divideSeaConnectedBlocks()
{
    LOGE("divideSeaConnectedBlocks 未实现");
}