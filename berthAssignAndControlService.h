#pragma once
#include "map.h"
#include "berth.h"
#include "goods.h"
#include "params.h"

// 对泊位聚类
class BerthAssignAndControlService
{
public:
    std::vector<std::vector<Berth>> clusters;   // 聚簇所得类
    std::vector<int> berthCluster; // 每个泊位所对应的类
public:
    // 初始化
    void initialize();
    // 设置参数，参数定义在子类里
    void setParameter(const Params &params);
    // 对泊位进行聚类
    void clusterBerths(const Map &map, std::vector<Berth> &berths);
    // 输出聚类结果
    void clusterResults();

private:
    std::vector<std::vector<Berth>>
    hierarchicalClustering(std::vector<Berth> &berths,
                           const std::vector<std::vector<int>> &distanceMatrix,
                           int numClusters);

    std::pair<int, int>
    findClosestClusters(const std::vector<std::vector<int>> &distanceMatrix,
                        const std::vector<bool> &merged);

    std::vector<std::vector<int>>
    inner_dist(std::vector<Berth> berths, const Map &map);
};
