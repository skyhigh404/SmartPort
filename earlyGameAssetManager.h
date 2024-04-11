#pragma once
#include "assetManager.h"

// 初始资金25000，机器人2000，轮船8000

// 陆地连通块
struct LandBlock
{
    int size; // 面积
    std::vector<Berth> berths;
};
// 海洋连通块
struct SeaBlock
{
    int size; // 面积
    std::vector<Point2d> deliveryLocations; // 交货点
};
struct LandSeaBlock
{
    int landSize;
    std::vector<Berth> berths;
    std::vector<Point2d> deliveryLocations;
    std::vector<Point2d> robotShops;
    std::vector<Point2d> shipShops;
};

// 追求长期利润
class EarlyGameAssetManager : public AssetManager
{
public:
    std::vector<PurchaseDecision> makePurchaseDecision(const Map &gameMap,
                                                       const std::vector<Goods> &goods,
                                                       const std::vector<Robot> &robots,
                                                       const std::vector<Ship> &ships,
                                                       const std::vector<Berth> &berths,
                                                       int currentFunds,
                                                       int currentTime) override;
    AssetManagerName getAssetManagerName() override
    {
        return AssetManagerName::EARLY_GAME_ASSET_MANAGER;
    }
    void setParameter(const Params &params) override;

    void init(const Map& map, std::vector<Berth> &berths) override; //初始化商店,可以在gamemanager中初始化
public:
    std::vector<Point2d> robotShops;
    std::vector<Point2d> shipShops;
    std::vector<LandBlock> landBlocks;
    std::vector<SeaBlock> seaBlocks;
    std::vector<LandSeaBlock> landseaBlocks;
    
    std::vector<int> purchasedRobotNum;
    std::vector<int> purchasedShipNum;

public:

private:
    // 超参数
    int maxRobotNum;                // 最多购买机器人数目
    int maxShipNum;                 // 最多购买船只数目
    std::vector<std::vector<int>> robotPurchaseAssign;
    std::vector<std::vector<int>> shipPurchaseAssign;
    int timeToBuyShip;              // 开始买船的时刻（除第一艘船外）
    int startNum;                   // 最初的数目（机器人、轮船）
    float landDistanceWeight;       // 对泊位价值评估时的陆地访问距离权重
    float deliveryDistanceWeight;   // 对泊位价值评估时的交货点访问距离权重
    bool CentralizedTransportation; // 开局是否集中搬货
    bool robotFirst;                // 先买机器人还是先买船，true为机器人、false为船

private:
    void divideLandConnectedBlocks(const std::vector<Berth> &berths, const Map &map);
    void divideSeaConnectedBlocks(const std::vector<Berth> &berths, const std::vector<Point2d> &deliveryLocations, const Map &map);
    void divideLandAndSeaConnectedBlocks(std::vector<Berth> &berths, const Map &map);
    void calBerthsEstimateValue(std::vector<Berth>& berths, const Map& map);
    bool needToBuyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
    bool needToBuyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds, int currentTime);
    Point2d buyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
    Point2d buyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
    Point2d getProperRobotShop(LandSeaBlock& l, const Map &gameMap);
    Point2d getProperShipShop(LandSeaBlock& l, const Map &gameMap);
    int getAssignId(Point2d shopPos, const std::vector<Berth> &berths);
};
