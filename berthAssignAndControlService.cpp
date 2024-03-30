#include "berthAssignAndControlService.h"

void BerthAssignAndControlService::clusterResults()
{
    for (int i=0;i<clusters.size();i++) {
        LOGI("class ", i, " 的包含的泊位数量：", clusters[i].size());
        for (int j=0;j<clusters[i].size();j++) {
            LOGI("泊位id：", clusters[i][j].id, ", 泊位位置：", clusters[i][j].pos);
        }
    }
}

void BerthAssignAndControlService::clusterBerths(const Map &map, std::vector<Berth> &berths)
{
    std::vector<bool> clustered(berths.size(), false);
    // 连通性聚类
    for (int i = 0; i < berths.size(); i++)
    {
        Berth &berth = berths[i];
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
            clusters.push_back(anotherClass);
        }
    }

    // for (int i=0;i<clusters.size();i++) {
    //     LOGI("class ", i, ' ', clusters[i].size());
    //     for (int j=0;j<clusters[i].size();j++) {
    //         LOGI(clusters[i][j].pos);
    //     }
    // }

    // 距离聚类
    while (clusters.size() < 5)
    {
        // LOGI(clusters.size());
        std::vector<std::vector<std::vector<int>>> inner_dist_grid(clusters.size());
        int max = 0, argmax = -1;
        // 找类内距最大的类进行拆分
        for (int i = 0; i < clusters.size(); i++)
        {
            inner_dist_grid[i] = inner_dist(clusters[i], map);
            int total_inner_dist = 0;
            for (int j = 0; j < clusters[i].size(); j++)
                for (int k = 0; k < clusters[i].size(); k++)
                    total_inner_dist += inner_dist_grid[i][j][k];
            if (total_inner_dist > max)
            {
                max = total_inner_dist;
                argmax = i;
            }
        }

        // LOGI(clusters[argmax].size(), ' ', inner_dist_grid[argmax].size());s
        std::vector<std::vector<Berth>> ret = hierarchicalClustering(clusters[argmax], inner_dist_grid[argmax], 2);
        clusters.erase(clusters.begin() + argmax);
        clusters.push_back(ret[0]);
        clusters.push_back(ret[1]);

        // LOGI("split");
        // for (int i=0;i<clusters.size();i++) {
        //     LOGI("class ", i, ' ', clusters[i].size());
        //     for (int j=0;j<clusters[i].size();j++) {
        //         LOGI(clusters[i][j].pos);
        //     }
        // }
    }

    // 更新berthCluster
    for (int i = 0; i < clusters.size(); i++)
    {
        for (int j = 0; j < clusters[i].size(); j++)
        {
            berthCluster[clusters[i][j].id] = i;
        }
    }
}

std::vector<std::vector<Berth>>
BerthAssignAndControlService::hierarchicalClustering(std::vector<Berth> &berths,
                        const std::vector<std::vector<int>> &distanceMatrix,
                        int numClusters)
{
    int N = distanceMatrix.size();
    std::vector<bool> merged(N, false);
    std::vector<std::vector<Berth>> clusters;

    // 初始化每个实体为一个独立的聚类
    for (int i = 0; i < N; ++i)
    {
        clusters.push_back({berths[i]});
    }

    // 重复合并直到只剩下指定数量的聚类
    // LOGI("合并");
    int numClusters_now = clusters.size();
    while (numClusters_now > numClusters)
    {
        // LOGI("clusters.size():",clusters.size());
        // 找到距离最近的两个聚类
        std::pair<int, int> closest = findClosestClusters(distanceMatrix, merged);
        int cluster1 = closest.first;
        int cluster2 = closest.second;

        // 合并聚类
        for (Berth entity : clusters[cluster2])
        {
            clusters[cluster1].push_back(entity);
        }
        // 标记已合并的聚类
        merged[cluster2] = true;

        // 删除被合并的聚类
        // clusters.erase(clusters.begin() + cluster2);
        numClusters_now--;
    }

    int erased = 0;
    for (int i = 0; i < N; i++)
    {
        if (merged[i])
        {
            clusters.erase(clusters.begin() + i - erased);
            erased++;
        }
    }

    return clusters;
}

std::pair<int, int> 
BerthAssignAndControlService::findClosestClusters(const std::vector<std::vector<int>> &distanceMatrix, const std::vector<bool> &merged)
{
    int minDist = INT_MAX;
    std::pair<int, int> closestClusters;
    int N = distanceMatrix.size();

    for (int i = 0; i < N; ++i)
    {
        if (!merged[i])
        {
            for (int j = i + 1; j < N; ++j)
            {
                if (!merged[j] && distanceMatrix[i][j] < minDist)
                {
                    minDist = distanceMatrix[i][j];
                    closestClusters = std::make_pair(i, j);
                }
            }
        }
    }
    return closestClusters;
}

std::vector<std::vector<int>> 
BerthAssignAndControlService::inner_dist(std::vector<Berth> berths, const Map &map)
{
    int n = berths.size();
    std::vector<std::vector<int>> grid(n, std::vector<int>(n));
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            grid[i][j] = map.berthDistanceMap.at(berths[i].id)[berths[j].pos.x][berths[j].pos.y];
        }
    }
    return grid;
}