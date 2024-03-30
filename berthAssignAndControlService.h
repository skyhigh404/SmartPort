#pragma once
#include "map.h"
#include "berth.h"
#include "goods.h"
#include "params.h"

class BerthAssignAndControlService
{
public:
    std::vector<int> berthCluster; // 每个泊位所对应的类
public:
    // 设置参数，参数定义在子类里
    void setParameter(const Params &params);
    // 对泊位进行聚类
    void clusterBerths(const Map &map, std::vector<Berth> &berths);
    // 更新所有货物被分配的泊位，通过更新 distsToBerths 实现
    void updateGoodsBerthAssignmets(std::vector<Goods> &goods, std::vector<Berth> &berths);

private:
    std::vector<std::vector<BerthID>>
    hierarchicalClustering(std::vector<Berth> &berths,
                           const std::vector<std::vector<int>> &distanceMatrix,
                           int numClusters);
};
