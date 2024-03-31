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

class GameManager : public BerthObserver
{
public:
    std::shared_ptr<RobotScheduler> robotScheduler;
    std::shared_ptr<ShipScheduler> shipScheduler;
    std::shared_ptr<RobotController> robotController;

    // 将下面的废弃
    // Scheduler *ShipScheduler;
    // Scheduler *RobotScheduler;

public:
    Map gameMap;
    CommandManager commandManager;
    SingleLaneManager singleLaneManager;
    std::vector<Robot> robots;
    std::vector<Ship> ships;
    std::vector<Goods> goods;
    std::vector<Berth> berths;
    int currentFrame;
    int currentMoney;
    // 统计
    int totalGetGoodsValue = 0;
    int skipFrame = 0;
    int finalFrame = -1; // 进入终局调度的帧数
    // int finalReadyFrame = -1;    // 进去终局调度前的准备帧数

public:
    GameManager() : gameMap(MAPROWS, MAPCOLS) {}
    void initializeGame();        // 读取初始化信息并初始化
    void initializeComponents();  // 初始化游戏各个类部件
    void processFrameData();      // 处理每帧的输入
    void update();                // 更新
    void outputCommands();        // 输出每帧的控制指令
    void robotControl();          // 运行机器人控制器
    void updateSingleLaneLocks(); // 维护单行路的锁

    void setShipScheduler(ShipScheduler *scheduler)
    {
        this->shipScheduler = scheduler;
    }
    void setRobotScheduler(RobotScheduler *scheduler)
    {
        this->robotScheduler = scheduler;
    }

    inline StageType nowStateType()
    {
        // 初始化
        if (finalFrame == -1)
        {
            // 最终帧计算公式
            // todo 可以调参,应该考虑机器人的路途代价
            // finalFrame = 15000 - 最大的泊位运输时间 * 3 - 最大的船舶容量 / 最小的泊位装货速度（装货时间） * 2 - 船舶移动的运输时间 - 缓冲时间

            int maxCapacity = -1, minVelocity = INT_MAX, maxTime = -1;
            for (auto &ship : ships)
                maxCapacity = std::max(maxCapacity, ship.capacity);
            for (auto &berth : berths)
                minVelocity = std::min(minVelocity, berth.velocity), maxTime = std::max(maxTime, berth.time);
            // finalFrame = 15000 - maxTime * 3 - static_cast<int>(maxCapacity/minVelocity) * 2 - 500;
            // finalFrame = 15000 - maxTime * 2 - static_cast<int>(maxCapacity/minVelocity) * 2 ;
            finalFrame = 15000 - maxTime * 3 - static_cast<int>(maxCapacity / minVelocity) * 2 - 500;
        }
        // LOGI("终局帧数：",finalFrame);
        // LOGI("当前帧数：",currentFrame);
        if (currentFrame < finalFrame)
        {
            return StageType::SIMPLE;
        }
        else
        {
            // LOGI("大于终局帧数");
            return StageType::FINAL;
        }
    }

private:
    // 处理泊位的状态发生变化
    void onBerthStatusChanged(int berthId, bool isEnabled) override;
};
