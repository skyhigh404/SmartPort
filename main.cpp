#include "gameManager.h"
#include "scheduler.h"
#include "log.h"
#include "pathFinder.h"

int main()
{
#ifdef DEBUG
    Log::initLog("../log/log.log");
#endif

    GameManager gameManager;
    SimpleTransportStrategy simpleTransportStrategy;
    gameManager.setScheduler(&simpleTransportStrategy);
    gameManager.initializeGame();

    // 测试 A* 算法
    // AStarPathfinder astar;
    // Point2d pos(47,10);
    // std::variant<Path, PathfindingFailureReason> path = astar.findPath(gameManager.robots[0].pos,pos,gameManager.gameMap);
    // std::vector<Point2d> *p = std::get_if<std::vector<Point2d>>(&path);
    // if(p){
    //     // for(const auto &pos : *p)
    //     //     LOGI("Position: ", pos);
    //     LOGI("Log A* from point ", gameManager.robots[0].pos," to ",pos);
    //     LOGI(gameManager.gameMap.drawMap(nullptr,nullptr, p, &gameManager.robots[0].pos, &pos));
    // }
    // else{
    //     PathfindingFailureReason *p = std::get_if<PathfindingFailureReason>(&path);
    //     LOGI("Find path error.", static_cast<int>(*p));
    // }

    LOGI("初始化完毕");
    while (1)
    {
        gameManager.processFrameData();
        gameManager.update();
        gameManager.outputCommands();
    }

    return 0;
}
