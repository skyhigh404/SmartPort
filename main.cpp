#include "gameManager.h"
#include "scheduler.h"
#include "log.h"
#include "pathFinder.h"
#include "finalShipScheduler.h"
#include <chrono>


using std::vector;

int main()
{
#ifdef DEBUG
    Log::initLog("../log/log.log");
#endif
    auto start = std::chrono::high_resolution_clock::now();

    GameManager gameManager;
    gameManager.initializeGame();
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    LOGI("Init finish, cost time: ",duration.count()," ms");
    LOGI("===================================================================");

    // 测试 A* 算法
    // VectorPosition startPos(49, 120, Direction::EAST);
    // VectorPosition targetPos(169, 99, Direction::EAST);
    // if (startPos == targetPos)
    //     LOGE("startPos==targetPos");

    // AStarPathfinder<VectorPosition, Map> astar;

    // LOGI("Start: ",startPos," target: ", targetPos);
    
    // start = std::chrono::high_resolution_clock::now();
    // std::variant<Path<VectorPosition>, PathfindingFailureReason> path = 
    //     astar.findPath(startPos, targetPos, gameManager.gameMap);
    // stop = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // LOGI("A* calculate take time: ",duration.count()," ms");

    // std::vector<VectorPosition> *p = std::get_if<std::vector<VectorPosition>>(&path);
    // if(p){
    //     LOGI("Log A* from point ", startPos," to ",targetPos);
    //     std::vector<Point2d> path2D;
    //     for(auto &point : *p){
    //         path2D.push_back(point.pos);
    //         // LOGI(point);
    //     }
        
    //     LOGI(gameManager.gameMap.drawMap(nullptr,nullptr, &path2D, &startPos.pos, &targetPos.pos));
    // }
    // else{
    //     PathfindingFailureReason *p = std::get_if<PathfindingFailureReason>(&path);
    //     LOGI("Find path error.", static_cast<int>(*p));
    // }
    // return 0;


    
    bool hasInitFinalShipScheduler = false; //判断终局船调度是否初始化
    while (1)
    {
        gameManager.processFrameData();
        // // 切换调度函数
        // switch (gameManager.nowStateType())
        // {
        // case StageType::BEGIN:
        //     //  不做切换
        //     break;
        // case StageType::FINAL:
        //     // 初始化
        //     if (!hasInitFinalShipScheduler){
        //         #ifdef DEBUG
        //         LOGI("进去终局船调度");
        //         #endif
        //         gameManager.shipScheduler = std::make_shared<FinalShipScheduler>(gameManager.berthAssignAndControlService.berthCluster,
        //         gameManager.berthAssignAndControlService.clusters);
        //         hasInitFinalShipScheduler = true;
        //     }
        //     break;
        // }
        auto start = std::chrono::steady_clock::now();
        gameManager.update();
        auto end = std::chrono::steady_clock::now();
        int time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        LOGI("本帧处理时间：",time,"ms");
        
        gameManager.outputCommands();
    }

    return 0;
}
