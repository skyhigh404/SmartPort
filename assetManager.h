#pragma once
#include "robot.h"
#include "ship.h"
#include "goods.h"
#include "map.h"
#include "berth.h"

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
    int quantity; // 购买数量
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
    virtual ~AssetManager() {}
};
