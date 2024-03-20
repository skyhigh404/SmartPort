#include "gameManager.h"
#include "scheduler.h"
#include "log.h"
#include "pathFinder.h"
#include <chrono>


using std::vector;

vector<vector<int>> inner_dist(vector<Berth> berths, Map &map)
{
    int n = berths.size();
    vector<vector<int>> grid(n, vector<int>(n));
    for (int i=0;i<n;i++) {
        for (int j=0;j<n;j++) {
            grid[i][j] = map.berthDistanceMap.at(berths[i].id)[berths[j].pos.x][berths[j].pos.y];
        }
    }
    return grid;
}

// 找到距离最近的两个聚类
std::pair<int, int> findClosestClusters(const vector<vector<int>>& distanceMatrix, const vector<bool>& merged) {
    int minDist = INT_MAX;
    std::pair<int, int> closestClusters;
    int N = distanceMatrix.size();

    for (int i = 0; i < N; ++i) {
        if (!merged[i]) {
            for (int j = i + 1; j < N; ++j) {
                if (!merged[j] && distanceMatrix[i][j] < minDist) {
                    minDist = distanceMatrix[i][j];
                    closestClusters = std::make_pair(i, j);
                }
            }
        }
    }
    return closestClusters;
}

// 层次聚类函数
vector<vector<Berth>> hierarchicalClustering(vector<Berth>& berths, const vector<vector<int>>& distanceMatrix, int numClusters) {
    int N = distanceMatrix.size();
    vector<bool> merged(N, false);
    vector<vector<Berth>> clusters;

    // 初始化每个实体为一个独立的聚类
    for (int i = 0; i < N; ++i) {
        clusters.push_back({berths[i]});
    }

    // 重复合并直到只剩下指定数量的聚类
    // LOGI("合并");
    int numClusters_now = clusters.size();
    while (numClusters_now > numClusters) {
        // LOGI("clusters.size():",clusters.size());
        // 找到距离最近的两个聚类
        std::pair<int, int> closest = findClosestClusters(distanceMatrix, merged);
        int cluster1 = closest.first;
        int cluster2 = closest.second;

        // 合并聚类
        for (Berth entity : clusters[cluster2]) {
            clusters[cluster1].push_back(entity);
        }
        // 标记已合并的聚类
        merged[cluster2] = true;

        // 删除被合并的聚类
        // clusters.erase(clusters.begin() + cluster2);
        numClusters_now--;
    }

    int erased = 0;
    for (int i=0;i<N;i++) {
        if (merged[i]) {
            clusters.erase(clusters.begin() + i - erased);
            erased++;
        }
    }

    return clusters;
}

vector<vector<Berth>> ClusteringBerths(vector<Berth> &berths, Map &map)
{
    vector<vector<Berth>> clusters;
    vector<bool> clustered(berths.size(), false);
    // 连通性聚类
    for (int i=0;i<berths.size();i++) {
        Berth& berth = berths[i];
        if (!clustered[i]) {
            vector<Berth> anotherClass;
            anotherClass.push_back(berths[i]);
            for (int j=i+1;j<berths.size();j++) {
                if (map.berthDistanceMap.at(i)[berths[j].pos.x][berths[j].pos.y] != INT_MAX) {
                    anotherClass.push_back(berths[j]);
                    clustered[j] = true;
                }
            }
            clusters.push_back(anotherClass);
        }
    }

    for (int i=0;i<clusters.size();i++) {
        LOGI("class ", i, ' ', clusters[i].size());
        for (int j=0;j<clusters[i].size();j++) {
            LOGI(clusters[i][j].pos);
        }
    }

    // 距离聚类
    while (clusters.size()<5) {
        LOGI(clusters.size());
        vector<vector<vector<int>>> inner_dist_grid(clusters.size());
        int max = 0, argmax = -1;
        // 找类内距最大的类进行拆分
        for (int i=0;i<clusters.size();i++) {
            inner_dist_grid[i] = inner_dist(clusters[i], map);
            int total_inner_dist = 0;
            for (int j=0;j<clusters[i].size();j++) for (int k=0;k<clusters[i].size();k++) total_inner_dist += inner_dist_grid[i][j][k];
            if (total_inner_dist > max) {
                max = total_inner_dist;
                argmax = i;
            }
        }

        // LOGI(clusters[argmax].size(), ' ', inner_dist_grid[argmax].size());s
        std::vector<std::vector<Berth>> ret = hierarchicalClustering(clusters[argmax], inner_dist_grid[argmax], 2);
        clusters.erase(clusters.begin() + argmax);
        clusters.push_back(ret[0]);
        clusters.push_back(ret[1]);

        LOGI("split");
        for (int i=0;i<clusters.size();i++) {
            LOGI("class ", i, ' ', clusters[i].size());
            for (int j=0;j<clusters[i].size();j++) {
                LOGI(clusters[i][j].pos);
            }
        }
    }
    return clusters;
}

int main()
{
#ifdef DEBUG
    Log::initLog("../log/log.log");
#endif

    GameManager gameManager;
    SimpleTransportStrategy simpleTransportStrategy;
    ImplicitEnumeration implicitEnumeration;
    FinalTransportStrategy finalTransportStrategy;
    gameManager.setShipScheduler(&simpleTransportStrategy);
    gameManager.setRobotScheduler(&simpleTransportStrategy);
    gameManager.initializeGame();
    LOGI("init finish");

    // std::vector<std::vector<Berth>> res = ClusteringBerths(gameManager.berths, gameManager.gameMap);
    // LOGI(res.size());
    // for (int i=0;i<res.size();i++) {
    //     LOGI("class ", i, ' ', res[i].size());
    //     for (int j=0;j<res[i].size();j++) {
    //         LOGI(res[i][j].pos);
    //     }
    // }
    // return 0;


    // 测试 A* 算法
    // AStarPathfinder astar;
    // Point2d pos(47,10);
    // LOGI("Start: ",gameManager.robots[0].pos," target: ", pos);
    
    // auto start = std::chrono::high_resolution_clock::now();
    // std::variant<Path, PathfindingFailureReason> path = astar.findPath(gameManager.robots[0].pos,pos,gameManager.gameMap);
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // LOGI("A* calculate take time: ",duration.count()," ms");

    // std::vector<Point2d> *p = std::get_if<std::vector<Point2d>>(&path);
    // if(p){
    //     LOGI("Log A* from point ", gameManager.robots[0].pos," to ",pos);
    //     LOGI(gameManager.gameMap.drawMap(nullptr,nullptr, p, &gameManager.robots[0].pos, &pos));
    // }
    // else{
    //     PathfindingFailureReason *p = std::get_if<PathfindingFailureReason>(&path);
    //     LOGI("Find path error.", static_cast<int>(*p));
    // }
    // return 0;

    LOGI("初始化完毕");
    while (1)
    {
        gameManager.processFrameData();
        // 调度变换
        if(gameManager.nowStateType() != gameManager.ShipScheduler->getSchedulerType()){
            switch (gameManager.nowStateType())
            {
            case StageType::FINAL:
                LOGI("进去船只终局调度");
                gameManager.setShipScheduler(&finalTransportStrategy);
                break;
            
            default:
                break;
            }
        }
        auto start = std::chrono::steady_clock::now();
        gameManager.update();
        auto end = std::chrono::steady_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        LOGI("本帧处理时间：",time,"ms");
        // if(time <=1){
        //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
        // }
        gameManager.outputCommands();
    }

    return 0;
}
