#pragma once
#include "robot.h"
#include "ship.h"
#include "goods.h"
#include "map.h"
#include "berth.h"
#include "params.h"

enum class AssetManagerName
{
    EARLY_GAME_ASSET_MANAGER,
    MID_GAME_ASSET_MANAGER,
    LATE_GAME_ASSET_MANAGER
};

enum class AssetType
{
    ROBOT,
    SHIP
};

class Asset
{
public:
    AssetType type;   // 资产类型
    int purchaseCost; // 购买成本
    int capacity;     // 资产能够携带或处理的最大货物量
    float efficiency; // 资产的效率

    Asset(AssetType type, int purchaseCost, int capacity, double efficiency)
        : type(type), purchaseCost(purchaseCost), capacity(capacity), efficiency(efficiency) {}
};

struct PurchaseDecision
{
    AssetType assetType;
    Point2d pos;  // 购买位置
    int quantity; // 购买数量
    int type;     // 购买的机器人类型：0（携带1个货物）、1（携带2个货物）
    int assignId; // 游戏初期对机器人集中分配的泊位id
};

class AssetManager
{
public:
    // 暴露给外部的接口
    // 判断是否要进行购买，购买多少
    virtual std::vector<PurchaseDecision> makePurchaseDecision(const Map &gameMap,
                                                               const std::vector<Goods> &goods,
                                                               const std::vector<Robot> &robots,
                                                               const std::vector<Ship> &ships,
                                                               const std::vector<Berth> &berths,
                                                               int currentFunds,
                                                               int currentTime) = 0;
    virtual AssetManagerName getAssetManagerName() = 0;
    virtual void setParameter(const Params &params) = 0;
    virtual void init(const Map& map, std::vector<Berth> &berths) = 0;
    virtual ~AssetManager() {}

public:
    const int robotPrice = 2000;
    const int shipPrice = 8000;
};
