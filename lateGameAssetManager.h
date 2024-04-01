#pragma once
#include "assetManager.h"

// 追求短期利润
class LateGameAssetManager : public AssetManager
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
        return AssetManagerName::LATE_GAME_ASSET_MANAGER;
    }
};
