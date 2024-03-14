#pragma once

#include <vector>
#include <string>

#include "utils.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "goods.h"
#include "berth.h"
#include "scheduler.h"
#include "commandManager.h"

class GameManager
{
public:
    Scheduler *scheduler;

public:
    Map gameMap;
    std::vector<Robot> robots;
    std::vector<Ship> ships;
    std::vector<Goods> goods;
    std::vector<Berth> berths;
    // TransportManager transportManager;
    int currentFrame;
    int currentMoney;
    CommandManager commandManager;

public:
    GameManager() : gameMap(MAPROWS, MAPCOLS)
    {
    }
    void initializeGame();   // 读取初始化信息并初始化
    void processFrameData(); // 处理每帧的输入
    void update();           // 更新
    void outputCommands();   // 输出每帧的控制指令

    void RobotControl(); // 控制机器人行为
    void setScheduler(Scheduler *scheduler)
    {
        this->scheduler = scheduler;
    }
};
