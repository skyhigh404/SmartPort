#include "gameManager.h"
#include <string>
#include <iostream>
#include "log.h"
#include "pathFinder.h"
#include <chrono>

using namespace std;
int Goods::number = 0;

void GameManager::initializeGame()
{
    // 读取地图
    string map_data;
    int robot_id = 0;
    for (int i = 0; i < MAPROWS; ++i)
    {
        cin >> map_data;
        for (int j = 0; j < MAPCOLS; ++j)
        {
            switch (map_data[j])
            {
            case '.':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SPACE);
                break;
            case '*':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SEA);
                break;
            case '#':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::OBSTACLE);
                break;
            case 'A':
                // 初始化机器人
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SPACE);
                this->robots.emplace_back(robot_id, Point2d(i, j));
                robot_id++;
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SPACE);
                break;
            case 'B':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::BERTH);
                break;
            default: 
                break;
            }
        }
    }
    // LOGI("Log init map info");
    // LOGI(this->gameMap.drawMap());

    // 初始化机器人
    // for (int i = 0; i < ROBOTNUMS; ++i)
    //     this->robots.emplace_back(i, Point2d(-1, -1));

    // 初始化泊位
    int id, x, y, time, velocity;
    for (int i = 0; i < BERTHNUMS; ++i)
    {
        cin >> id >> x >> y >> time >> velocity;
        this->berths.emplace_back(id, Point2d(x, y), time, velocity);
    }
    // LOGI("print berth init info");
    // for (const auto &berth : this->berths)
    //     LOGI("ID: ", berth.id, " POS: ", berth.pos, " time: ", berth.time, " velocity: ", berth.velocity);

    // 初始化船舶
    int capacity;
    cin >> capacity;
    for (int i = 0; i < SHIPNUMS; ++i)
    {
        this->ships.emplace_back(i, capacity);
    }
    // for (int i = 0; i < SHIPNUMS; ++i)
    // {
    //     LOGI("Ship ", this->ships[i].id," capacity: ", this->ships[i].capacity);
    // }
    
    // 计算地图上每个点到泊位的距离
    for (const auto &berth : this->berths)
    {
        vector<Point2d> positions;
        // 泊位大小 4x4
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                positions.push_back(berth.pos + Point2d(i, j));
    
        this->gameMap.computeDistancesToBerthViaBFS(berth.id, positions);
    }

    // 判断机器人是否位于死点
    for(auto& robot : this->robots){
        bool is_isolated = true;
        for(const auto& berth : this->berths){
            if(this->gameMap.isBerthReachable(berth.id,robot.pos)){
                is_isolated = false;
                break;
            }
        }
        // 孤立机器人
        if (is_isolated)
            robot.status = DEATH;
    }

    // LOGI("Log berth 0 BFS map.");
    // LOGI(Map::drawMap(this->gameMap.berthDistanceMap[0],12));

    string ok;
    cin >> ok;
    if(ok == "OK"){
        LOGI("Init complete.");
        cout << "OK" << std::endl;
    }
    else{
        LOGE("Init fail!");
    }
}

void GameManager::processFrameData()
{
    // LOGI("processFrameData.");
    int newItemCount;
    int goodsX, goodsY, value;
    int carrying, robotX, robotY, robotState;
    int shipState, berthId;
    // 如果读取到了 EOF，则结束
    if (cin.eof())
    {
        exit(0);
    }

    cin >> this->currentFrame >> this->currentMoney;
    // 货物生命周期维护
    for (auto& good : goods)
        good.TTL = std::max(good.TTL - (currentFrame - good.initFrame),-1);
    // 读取新增货物
    cin >> newItemCount;
    while (newItemCount--)
    {
        cin >> goodsX >> goodsY >> value;
        this->goods.emplace_back(Point2d(goodsX, goodsY), value, currentFrame);
    }
    // 读取机器人状态
    for (int i = 0; i < ROBOTNUMS; ++i)
    {
        cin >> carrying >> robotX >> robotY >> robotState;
        this->robots[i].carryingItem = carrying;
        this->robots[i].pos.x = robotX;
        this->robots[i].pos.y = robotY;
        this->robots[i].state = robotState;
    }
    // 读取船舶状态
    for (int i = 0; i < SHIPNUMS; ++i)
    {
        cin >> shipState >> berthId;
        this->ships[i].state = shipState;
        this->ships[i].berthId = berthId;
    }
    // 确认已接收完本帧的所有数据
    string ok;
    cin >> ok;

    // for (int i = 0; i < ROBOTNUMS; ++i)
    // {
    //     LOGI("Robot: ", this->robots[i].id, " position: ", this->robots[i].pos, " state: ", this->robots[i].state);
    // }
}

void GameManager::update()
{   
    auto start = std::chrono::steady_clock::now();
    
    bool robotDebugOutput = true;
    AStarPathfinder pathfinder;
    for (int i=0;i<robots.size();i++) {
        // 机器人寻路路径为空 && 不位于死点
        if (robots[i].path.empty() && robots[i].status != DEATH) {
            // 调用调度器来获取机器人的目的地
            if (robotDebugOutput) LOGI(i, "寻路中");
            Action action = this->scheduler->scheduleRobot(robots[i], gameMap, goods, berths, robotDebugOutput);
            if (action.type==FAIL) continue;
            std::variant<Path, PathfindingFailureReason> path = pathfinder.findPath(robots[i].pos, action.desination, gameMap);
            if (std::holds_alternative<Path>(path)) {
                if (robotDebugOutput) LOGI(i, "寻路成功");
                robots[i].path = std::get<Path>(path);
                if (robotDebugOutput) LOGI(i,"路径长度：",robots[i].path.size());
            }
            else {
                if (robotDebugOutput) LOGI(i, "寻路失败");
                robots[i].path = Path();
                robots[i].status = IDLE;
            }
            
        }
        if (!robots[i].path.empty()) {
            // 机器人有路径，继续沿着路径移动
            const std::string temp = robots[i].moveWithPath();
            if (robotDebugOutput) LOGI(i, "移动中:",temp);
            commandManager.addRobotCommand(temp);
            if (robots[i].path.empty()) {
                if (robots[i].carryingItem==0) {
                    if (robotDebugOutput) LOGI(i, "拿起货物");
                    commandManager.addRobotCommand(robots[i].get());
                    robots[i].carryingItem = 1;
                    robots[i].carryingItemId = robots[i].targetid;
                    // 更新货物TTL
                    goods[robots[i].targetid].TTL = INT_MAX;
                }
                else {
                    if (robotDebugOutput) LOGI(i, "放下货物");
                    for (int l=0;l<berths[robots[i].targetid].unreached_goods.size();l++) {
                        if (berths[robots[i].targetid].unreached_goods[l].id==goods[robots[i].carryingItemId].id) {
                            berths[robots[i].targetid].unreached_goods.erase(berths[robots[i].targetid].unreached_goods.begin() + l);
                        }
                    }
                    berths[robots[i].targetid].reached_goods.push_back(goods[robots[i].carryingItemId]);
                    commandManager.addRobotCommand(robots[i].pull());
                    robots[i].carryingItem = 0;
                }
            }
        }
    }
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // LOGI("--------------------------------------------------------",duration.count(),"ms");

    auto ship_start = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, Action>> ShipActions = this->scheduler->scheduleShips(ships, berths, false);
    auto ship_end = std::chrono::high_resolution_clock::now();
    // LOGI("调度船只时长:",std::chrono::duration_cast<std::chrono::milliseconds>(ship_end - ship_start).count(),"ms");

    //CommandManager.shipCommands
    for (int i=0;i<ShipActions.size();i++) {
        int ship_id = ShipActions[i].first;
        Action ship_action = ShipActions[i].second;
        // 去虚拟点
        if (ship_action.type==DEPART_BERTH) {
            // LOGI(ship_id,"前往虚拟点");
            commandManager.addShipCommand(ships[ship_id].go());
        }
        // 去泊位
        if (ship_action.type==MOVE_TO_BERTH) {
            // LOGI(ship_id,"分配去泊位",ship_action.targetId);
            commandManager.addShipCommand(ships[ship_id].moveToBerth(ship_action.targetId));
        }
    }
}

void GameManager::outputCommands()
{
    
    commandManager.outputCommands();
    commandManager.clearCommands();
}
