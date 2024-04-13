#pragma once

#include <vector>
#include <string>
#include <memory>
#include "utils.h"
#include "map.h"
#include "robot.h"
#include "ship.h"
#include "goods.h"
#include "berth.h"
#include "scheduler.h"
#include "commandManager.h"
#include "robotController.h"
#include "shipController.h"
#include "assetManager.h"
#include "berthAssignAndControlService.h"
#include "singleLaneManager.h"
#include "seaSingleLaneManager.h"

enum class StageType
{
    BEGIN,
    FINAL
};

class GameManager : public BerthObserver
{
public:
    std::shared_ptr<RobotScheduler> robotScheduler;
    std::shared_ptr<ShipScheduler> shipScheduler;
    std::shared_ptr<RobotController> robotController;
    std::shared_ptr<ShipController> shipController;
    std::shared_ptr<AssetManager> assetManager;

public:
    Map gameMap;
    CommandManager commandManager;
    SingleLaneManager singleLaneManager;
    SeaSingleLaneManager seaSingleLaneManager;
    ParamReader paramReader;
    std::vector<Robot> robots;
    std::vector<Ship> ships;
    std::vector<Goods> goods;
    std::vector<Berth> berths;
    BerthAssignAndControlService berthAssignAndControlService;
    int currentFrame;
    int currentMoney;
    // 统计
    int totalGetGoodsValue = 0;
    int skipFrame = 0;
    int finalFrame = -1;                                                 // 进入终局调度的帧数
    std::vector<std::vector<int>> goodsExpiredMap;                    // 存储每个地点生成的货物数目
    std::unordered_map<std::string, int> generateGoodsValueDistribution; // 用于存储不同价值区间的货物数量
    std::unordered_map<std::string, int> expiredGoodsValueDistribution;  // 用于存储过期不同价值区间的货物数量
    std::unordered_map<std::string, int> getGoodsValueDistribution;      // 用于存储过期不同价值区间的货物数量

    int SHIP_STILL_FRAMES_LIMIE; // 船阻塞帧数限制

public:
    GameManager() : gameMap(MAPROWS, MAPCOLS) {}
    void initializeGame();                                                              // 读取初始化信息并初始化
    void initializeComponents();                                                        // 初始化游戏各个类部件
    void processFrameData();                                                            // 处理每帧的输入
    void update();                                                                      // 更新
    void outputCommands();                                                              // 输出每帧的控制指令
    void robotControl();                                                                // 运行机器人控制器
    void shipControl();                                                                 // 运行船控制器
    void assetControl();                                                                // 运行资产管理
    void updateSingleLaneLocks();                                                       // 维护单行路的锁
    void logStatisticsInfo();                                                           // 结束前输出统计信息
    void statisticGoods(int value, std::unordered_map<std::string, int> &statisticMap); // 统计货物情况

    StageType nowStateType();

private:
    // 处理泊位的状态发生变化
    void onBerthStatusChanged(int berthId, bool isEnabled) override;
};
