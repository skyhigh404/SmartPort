#include "gameManager.h"
#include <string>
#include <iostream>

using namespace std;

void GameManager::initializeGame()
{
    // 读取地图
    string map_data;
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
                // this->robots.emplace_back(robotID++, Point2d(i, j));
                break;
            case 'B':
                break;
            default:
                break;
            }
        }
    }

    // 初始化机器人
    for (int i = 0; i < ROBOTNUMS; ++i)
        this->robots.emplace_back(i, Point2d(-1, -1));

    // 初始化泊位
    int id, x, y, time, velocity;
    for (int i = 0; i < BERTHNUMS; ++i)
    {
        cin >> id >> x >> y >> time >> velocity;
        this->berths.emplace_back(id, Point2d(x, y), time, velocity);
    }

    // 初始化船舶
    int capacity;
    cin >> capacity;
    for (int i = 0; i < SHIPNUMS; ++i)
    {
        this->ships.emplace_back(i, capacity);
    }

    string ok;
    cin >> ok;

    // printf("OK\n");
    // fflush(stdout);
}

void GameManager::processFrameData()
{
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
}

void GameManager::update()
{
    this->scheduler->scheduleRobots(robots, gameMap, goods, commandManager);
    this->scheduler->scheduleShips(ships, berths, commandManager);
}

void GameManager::outputCommands()
{
    commandManager.outputCommands();
    commandManager.clearCommands();
}
