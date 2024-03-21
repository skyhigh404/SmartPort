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
#include "robotController.h"
#include <memory>


#include "singleLaneManager.h"

class GameManager
{
public:
    Scheduler *ShipScheduler;
    Scheduler *RobotScheduler;

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
    std::shared_ptr<RobotController> robotController;
    // 统计
    int totalGetGoodsValue = 0;
    int skipFrame = 0;

    int finalFrame = -1; //进入终局调度的帧数
    // int finalReadyFrame = -1;    // 进去终局调度前的准备帧数

public:
    SingleLaneManager singleLaneManager;

    GameManager() : gameMap(MAPROWS, MAPCOLS)
    {
    }
    void initializeGame();   // 读取初始化信息并初始化
    void processFrameData(); // 处理每帧的输入
    void update();           // 更新
    void outputCommands();   // 输出每帧的控制指令
    void RobotControl();
    void robotControl();
    void updateSingleLaneLocks();   // 维护单行路的锁

    void setShipScheduler(Scheduler *scheduler)
    {
        this->ShipScheduler = scheduler;
    }

    inline StageType nowStateType(){
        // 初始化
        if(finalFrame == -1)
        {
            // 最终帧计算公式
            // todo 可以调参,应该考虑机器人的路途代价
            // finalFrame = 15000 - 最大的泊位运输时间 * 3 - 最大的船舶容量 / 最小的泊位装货速度（装货时间） * 2 - 缓冲时间
            
            int maxCapacity = -1,minVelocity = INT_MAX,maxTime = -1;
            for(auto &ship : ships) maxCapacity = std::max(maxCapacity,ship.capacity);
            for(auto &berth : berths) minVelocity = std::min(minVelocity,berth.velocity),maxTime = std::max(maxTime, berth.time);
            finalFrame = 15000 - maxTime * 3 - static_cast<int>(maxCapacity/minVelocity) * 2; 
        }
        // LOGI("终局帧数：",finalFrame);
        // LOGI("当前帧数：",currentFrame);
        if(currentFrame < finalFrame){
            return StageType::SIMPLE;
        }
        else{
            // LOGI("大于终局帧数");
            return StageType::FINAL;
        }
    }

    void setRobotScheduler(Scheduler *scheduler)
    {
        this->RobotScheduler = scheduler;
    }
};
