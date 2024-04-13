#pragma once
#include "assetManager.h"

// 倾向于平衡增长和风险
class MidGameAssetManager : public AssetManager
{
private:
    // 预测购买特定数量和类型的资源（机器人或船舶）能带来的潜在收益
    float predictProfitFromPurchase(const AssetType &type, int quantity);
    // 计算投资回报率（ROI），帮助决策是否进行购买。
    float calculateROI(const Asset &asset, int quantity, int futureGoodsValue, int futureGoodsNum);
    // 预测未来产生的货物数量和价值
    std::pair<int, int> predictFutureGoods(const std::vector<Goods> &goods, int currentTime);
    // 预测未来泊位堆积货物的情况
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
        return AssetManagerName::MID_GAME_ASSET_MANAGER;
    }
};
