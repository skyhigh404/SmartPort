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
    // std::vector<Berth> berths; // 交货点
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

public:
    std::vector<Point2d> robotShops;
    std::vector<Point2d> shipShops;
    std::vector<LandBlock> landBlocks;
    std::vector<SeaBlock> seaBlocks;
    

public:
    void init(const Map& map, const std::vector<Berth> &berths); //初始化商店,可以在gamemanager中初始化

private:
    // 超参数
    int maxRobotNum;    // 最多购买机器人数目
    int maxShipNum;     // 最多购买船只数目

private:
    void divideLandConnectedBlocks(const std::vector<Berth> &berths, const Map &map);
    void divideSeaConnectedBlocks(); //TODO：根据交货点划分
    bool needToBuyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
    bool needToBuyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
    void buyRobot(const std::vector<Robot> &robots, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
    bool buyShip(const std::vector<Ship> &ships, const std::vector<Goods> &goods, const Map &gameMap, int currentFunds);
};
