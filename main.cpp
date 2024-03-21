#include "gameManager.h"
#include "scheduler.h"
#include "log.h"
#include "pathFinder.h"
#include <chrono>


using std::vector;

int main()
{
#ifdef DEBUG
    Log::initLog("../log/log.log");
#endif

    GameManager gameManager;
    SimpleTransportStrategy simpleTransportStrategy;
    ImplicitEnumeration implicitEnumeration;
    FinalTransportStrategy finalTransportStrategy;
    FinalClusterTransportStrategy finalClusterTransportStrategy;
    gameManager.setShipScheduler(&simpleTransportStrategy);
    gameManager.setRobotScheduler(&simpleTransportStrategy);
    gameManager.initializeGame();
    // 初始化聚类
    simpleTransportStrategy.initCluster(gameManager.berths,gameManager.gameMap);
    finalTransportStrategy.initCluster(gameManager.berths,gameManager.gameMap);
    implicitEnumeration.initCluster(gameManager.berths,gameManager.gameMap);
    finalClusterTransportStrategy.initCluster(gameManager.berths,gameManager.gameMap);
    // simpleTransportStrategy.assignRobots(gameManager.robots, gameManager.gameMap);
    // LOGI("1");
    // for (int i=0;i<gameManager.robots.size();i++) {
    //     LOGI(simpleTransportStrategy.assignment[i]);
    // }
    // return 0;

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
                // // 货物价值最大化终局调度
                // gameManager.setShipScheduler(&finalTransportStrategy);
                // 聚类均衡终局调度
                gameManager.setShipScheduler(&finalClusterTransportStrategy);
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
