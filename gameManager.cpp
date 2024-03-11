#include "gameManager.h"
#include <string>
#include <iostream>
#include "log.h"

using namespace std;
int Goods::number = 0;

void GameManager::initializeGame()
{
    // 读取地图
    string map_data;
    int robot_id = 1;
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
                this->robots.emplace_back(robot_id, Point2d(i, j));
                robot_id++;
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
    LOGI("print berth init info");
    for (const auto &berth : this->berths)
        LOGI("ID: ", berth.id, " POS: ", berth.pos, " time: ", berth.time, " velocity: ", berth.velocity);

    // 初始化船舶
    int capacity;
    cin >> capacity;
    for (int i = 0; i < SHIPNUMS; ++i)
    {
        this->ships.emplace_back(i, capacity);
    }
    for (int i = 0; i < SHIPNUMS; ++i)
    {
        LOGI("Ship ", this->ships[i].id," capacity: ", this->ships[i].capacity);
    }
    
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
    // 读取新增货物
    cin >> newItemCount;
    while (newItemCount--)
    {
        cin >> goodsX >> goodsY >> value;
        this->goods.emplace_back(Point2d(goodsX, goodsY), value);
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
    this->scheduler->scheduleRobots(robots, gameMap, goods, berths);
    this->scheduler->scheduleShips(ships, berths);
    // commandManager，获取命令
    // todo 输出指令
    // CommandManager.robotCommands
}

void GameManager::outputCommands()
{
    
    commandManager.outputCommands();
    commandManager.clearCommands();
}
