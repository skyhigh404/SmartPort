#include "gameManager.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <chrono>
#include "log.h"
#include "greedyRobotScheduler.h"
#include "greedyShipScheduler.h"
#include "finalShipScheduler.h"

using namespace std;
int Ship::capacity = 0;
int Goods::count = 0;
int Berth::totalLoadGoodnum = 0;
int Berth::maxLoadGoodNum = 0;
int Berth::deliverGoodNum = 0;
int CURRENT_FRAME = 0;  //当前帧数
MapFlag MAP_TYPE = MapFlag::ERROR;  // LABYRINTH: 图二、迷宫;NORMAL:图一、正常图;UNKNOWN；图三未知图;ERROR :默认值
int last_assign = 0;
int canUnload(Berth& berth, Point2d pos) {
    int x = pos.x-berth.pos.x, y=pos.y-berth.pos.y;
    if (x<0 || x>3 || y<0 || y>3) {
        return 0;
    }
    else 
        return 1;
}

std::vector<int> berthDistrubtGoodNumCount;
std::vector<int> berthDistrubtGoodValueCount;

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
            // if(i == j) diagonal += map_data[j];
            switch (map_data[j])
            {
            case '.':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SPACE);
                break;
            case '>':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::MAIN_ROAD);
                break;
            case '*':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SEA);
                break;
            case '~':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SEA_LANE);
                break;
            case '#':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::OBSTACLE);
                break;
            case 'R':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::ROBOT_SHOP);
                this->gameMap.robotShops.emplace_back(i, j);
                break;
            case 'S':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::SHIP_SHOP);
                this->gameMap.shipShops.emplace_back(i, j);
                break;
            case 'B':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::BERTH);
                break;
            case 'K':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::MOORING_AREA);
                break;
            case 'C':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::HYBRID);
                break;
            case 'c':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::HYBRID_LANE);
                break;
            case 'T':
                this->gameMap.setCell(i, j, MapItemSpace::MapItem::DELIVERY_POINT);
                this->gameMap.deliveryLocations.push_back({i,j});
                break;
            default:
                break;
            }
        }
    }
    this->gameMap.readOnlyGrid = this->gameMap.grid;

    // LOGI("Log init map info");
    // LOGI(this->gameMap.drawMap());

    // 初始化泊位
    int id, x, y, time, velocity, berthNum;
    cin >> berthNum;
    for (int i = 0; i < berthNum; ++i)
    {
        cin >> id >> x >> y >> velocity;
        this->berths.emplace_back(id, Point2d(x, y), velocity);
    }
    berthDistrubtGoodNumCount = std::vector<int>(this->berths.size(), 0);
    berthDistrubtGoodValueCount = std::vector<int>(this->berths.size(), 0);
    LOGI("print berth init info");
    for (const auto &berth : this->berths)
        LOGI("ID: ", berth.id, " POS: ", berth.pos, " velocity: ", berth.velocity);

    // 初始化船舶
    cin >> Ship::capacity;
    // for (int i = 0; i < SHIPNUMS; ++i)
    // {
    //     LOGI("Ship ", this->ships[i].id," capacity: ", this->ships[i].capacity);
    // }

    // 初始化数据读取完成
    // 进行其他部件的初始化
    initializeComponents();
    

    string ok;
    cin >> ok;
    if (ok == "OK")
    {
        LOGI("Init complete.");
        cout << "OK" << std::endl;
    }
    else
    {
        LOGE("Init fail!");
    }
    
    // 打印单行路
    // LOGI("单行路数量：",this->singleLaneManager.singleLanes.size());
    // int singleLaneSize = this->singleLaneManager.singleLanes.size();
    // LOGI(this->gameMap.drawMap(nullptr, nullptr, nullptr, nullptr, nullptr));
    // LOGI("Log berth 0 BFS map.");
    // LOGI(Map::drawMap(this->gameMap.berthDistanceMap[9],12));
    // exit(0);
}

void GameManager::initializeComponents()
{
    // 1. 让地图实时跟踪机器人位置
    // for (Robot &robot : this->robots)
    //     this->gameMap.robotPosition.push_back(robot.pos);

    // 2. 使用 BFS 计算地图上每个点到泊位的距离
    for (auto &berth : this->berths)
    {
        vector<Point2d> positions;
        // 泊位大小 4x4
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                positions.push_back(berth.pos + Point2d(i, j));

        this->gameMap.computeDistancesToBerthViaBFS(berth.id, positions);
        this->gameMap.computeMaritimeBerthDistanceViaBFS(berth.id, positions);
        berth.distsToDelivery = this->gameMap.initializeBerthToDeliveryDistances(berth.id);
    }

    // 预先计算海图航线
    std::vector<Point2d> nodes;
    for (const auto &berth : this->berths)
        nodes.push_back(berth.pos);
    nodes.insert(nodes.end(),
                this->gameMap.deliveryLocations.begin(),
                this->gameMap.deliveryLocations.end());
    nodes.insert(nodes.end(),
                this->gameMap.shipShops.begin(),
                this->gameMap.shipShops.end());
    for(int i = 0; i < nodes.size(); ++i)
    {
        for(int j = i+1; j < nodes.size(); ++j)
        {
            VectorPosition startVP(nodes[i], Direction::EAST);
            VectorPosition targetVP(nodes[j], Direction::EAST);
            SeaRoute::findPath(this->gameMap, startVP, targetVP);
        }
    }

    // 3. 判断机器人是否 DEATH 状态
    for (auto &robot : this->robots)
    {
        bool is_isolated = true;
        for (const auto &berth : this->berths)
        {
            if (this->gameMap.isBerthReachable(berth.id, robot.pos))
            {
                is_isolated = false;
                break;
            }
        }
        // 孤立机器人
        if (is_isolated)
        {
            robot.status = DEATH;
            LOGI("死機器人:", robot.id);
        }
    }
    // 4. 对所有泊位注册gameManager作为观察者
    for (auto &berth : berths)
        berth.registerObserver(this);
    // 5. 初始化单行路
    this->singleLaneManager.init(gameMap);
    // 6. 判断地图类型，后续封装在其他函数中实现
    MAP_TYPE = MapFlag::NORMAL;
    // 7. 读取参数
    Params params(MAP_TYPE);
    // 8. 初始化 RobotController
    this->robotController = std::make_shared<RobotController>(this->robots);
    // 9. 对泊位进行聚类
    this->berthAssignAndControlService.initialize(this->gameMap,this->berths);
    std::vector<int> &berthCluster = this->berthAssignAndControlService.berthCluster;
    std::vector<std::vector<Berth>> &clusters = this->berthAssignAndControlService.clusters;
    // 10. 注册机器人调度函数
    robotScheduler = std::make_shared<GreedyRobotScheduler>(clusters, berthCluster);
    // 11. 注册船舶调度函数
    shipScheduler = std::make_shared<GreedyShipScheduler>();
    // 12. 对机器人调度函数更新Params
    this->robotScheduler->setParameter(params);
    // 13. 对船舶调度函数更新Params
    this->shipScheduler->setParameter(params);
}

void GameManager::processFrameData()
{
    int newItemCount;
    int goodsX, goodsY, value;
    int carrying, robotX, robotY, robotState;
    // int shipState, berthId;
    // 如果读取到了 EOF，则结束
    if (cin.eof())
    {
        exit(0);
    }
    
    // 清除临时障碍
    gameMap.clearTemporaryObstacles();

    cin >> this->currentFrame >> this->currentMoney;
    int skipFrame = this->currentFrame - CURRENT_FRAME - 1;
    this->skipFrame += skipFrame;
    if(skipFrame)
        LOGW("跳帧: ", skipFrame);
    CURRENT_FRAME = this->currentFrame;
    LOGI("=======================================新的一帧====================================");
    // 货物生命周期维护
    for (auto& good : goods){
        // todo 边界条件
        if(good.TTL != INT_MAX && good.TTL >= 0){
            // LOGI(good.id,",initFrame:",good.initFrame,",currentFrame:",currentFrame,",TTL:",good.TTL);
            good.TTL = std::max(1000-(currentFrame - good.initFrame),-1);
        }
    }
    // 读取变化货物
    // TODO: 使用Map::computePointToBerthsDistances计算货物到泊位距离
    cin >> newItemCount;
    while (newItemCount--)
    {
        cin >> goodsX >> goodsY >> value;
        // 金额为 0 表示上一帧被拿取或者该帧消失
        if (value==0) continue;
        Goods good(Point2d(goodsX, goodsY), value, currentFrame);
        good.distsToBerths = gameMap.computePointToBerthsDistances(Point2d(goodsX, goodsY));
        this->goods.push_back(good);
        
        int tempGoodDistrubtID = this->gameMap.getNearestBerthID(Point2d(goodsX, goodsY));
        if(tempGoodDistrubtID>=0 && tempGoodDistrubtID<berths.size()) {
            berthDistrubtGoodNumCount[tempGoodDistrubtID]++;
            berthDistrubtGoodValueCount[tempGoodDistrubtID] += value;
        }
    }
    // 读取机器人状态
    int robotNum, robotId;
    std::cin >> robotNum;
    for (int i = 0; i < robotNum; ++i)
    {
        cin >> robotId >> carrying >> robotX >> robotY;
        // 创建机器人
        if (i >= this->robots.size()) this->robots.emplace_back(Robot(robotId, Point2d(robotX, robotY)));
        this->robots[robotId].carryingItem = carrying;
        this->robots[robotId].pos.x = robotX;
        this->robots[robotId].pos.y = robotY;

        // 暂时处理程序认为机器人放下了货物并且分配了下一个货物的id，但是判题器认为机器人还拿着上一个货物的情况
        if (this->robots[robotId].carryingItem == 0)
        {
            // 强行初始化机器人状态
            this->robots[robotId].carryingItemId = -1;
        }
        // 检查是否要 pop path
        this->robots[robotId].updatePath();
    }

    // 读取船舶状态
    int shipNum, shipId, goodsCount, shipX, shipY, direction, shipState;
    std::cin >> shipNum;
    for (int i = 0; i < shipNum; ++i)
    {
        cin >> shipId >> goodsCount >> shipX >> shipY >> direction >> shipState;
        if (i >= this->ships.size())
            this->ships.emplace_back(Ship(shipId));
        this->ships[shipId].goodsCount = goodsCount;
        this->ships[shipId].locAndDir.pos.x = shipX;
        this->ships[shipId].locAndDir.pos.y = shipY;
        this->ships[shipId].locAndDir.direction = static_cast<Direction>(direction);
        this->ships[shipId].state = shipState;
    }
    // 确认已接收完本帧的所有数据
    string ok;
    cin >> ok;

    // 初始化泊位货物状态
    for(auto &berth : berths){
        berth.unreached_goods = std::vector<Goods>();
        // berth.reached_goods = std::vector<Goods>();
    }
}


void GameManager::robotControl()
{
    bool robotDebugOutput = true;
    // 机器人状态更新
    for (Robot& robot:robots) {
        if (robot.status==DEATH) continue;

        // 机器人状态更新
        if (robot.carryingItem==0) robot.status = MOVING_TO_GOODS;
        else robot.status = RobotStatus::MOVING_TO_BERTH;
    }

    // 对所有可能的机器人执行取货或放货指令，更新状态
    for (Robot& robot : robots) {
        if (robot.status==DEATH) continue;
        if (robot.status==MOVING_TO_GOODS && robot.targetid!=-1 && robot.pos == goods[robot.targetid].pos) {
            if (goods[robot.targetid].TTL>0) {
                commandManager.addRobotCommand(robot.get());
                robot.carryingItem = 1;
                robot.carryingItemId = robot.targetid;
                robot.status = RobotStatus::MOVING_TO_BERTH;
                goods[robot.targetid].TTL = INT_MAX;
                robot.targetid = -1;
                // Berth::maxLoadGoodNum += 1;
            }
            // 货物过期
            else {
                robot.status = MOVING_TO_GOODS;
                robot.targetid = -1;
                continue;
            }
        }
        else if(robot.status==MOVING_TO_BERTH && robot.targetid!=-1) {
            Berth &berth = berths[robot.targetid];
            // LOGI(robot);
            if (canUnload(berth, robot.pos)) {
                LOGI("機器人",robot.id,"放貨 ");
                commandManager.addRobotCommand(robot.pull());
                int x = robot.pos.x-berth.pos.x, y=robot.pos.y-berth.pos.y;
                if(currentFrame < 15000 - berth.timeToDelivery()){
                    Berth::maxLoadGoodNum += 1;
                    totalGetGoodsValue += goods[robot.carryingItemId].value;
                    berth.reached_goods.push_back(goods[robot.carryingItemId]);
                    goods[robot.carryingItemId].status = 3;
                }
                LOGI("机器人效率统计, 当前时间, ",currentFrame,", robotID, ", robot.id, ", goodValue, ", goods[robot.carryingItemId].value, ", berthID, ", berth.id);
                goods[robot.carryingItemId].status = 3;
                robot.status = MOVING_TO_GOODS;
                robot.carryingItem = 0;
                robot.carryingItemId = -1;
                robot.targetid = -1;
                robot.path = Path<Point2d>();
            }
            else {
                // robot.targetid = -1;
            }
        }
    }

    if (this->nowStateType()==StageType::FINAL) {
        LOGI("機器人調度進入終局");
        for (Robot& robot:robots) {
            if (robot.targetid==-1) continue;
            if ( (robot.status==MOVING_TO_BERTH && berths[robot.targetid].isEnable()==false) || (robot.status==MOVING_TO_GOODS && berths[goods[robot.targetid].distsToBerths[0].first].isEnable()==false)) {
                robot.targetid = -1;
                robot.destination = Point2d(-1,-1);
                robot.path = Path<Point2d>();
            }
        }
        LOGI("機器人調度進入終局");
    }
    
    auto start = std::chrono::steady_clock::now();
    // 对所有需要调度的机器人进行调度
    this->robotScheduler->scheduleRobots(gameMap, robots, goods, berths, currentFrame);

    auto end = std::chrono::steady_clock::now();
    LOGI("scheduleRobots时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");
    // LOGI("機器人調度完畢");

    // 执行动作
    robotController->runController(gameMap, this->singleLaneManager);
    // LOGI("機器人尋路完畢");
    // 维护单行路的锁
    updateSingleLaneLocks();
    
    // 输出指令
    for (Robot& robot : robots) {
        if (robot.status==DEATH) continue;
        if (!robot.path.empty()) {
            string command = robot.movetoNextPosition();
            // if (robotDebugOutput) LOGI(robot.id, "向货物移动中:", command, " 路径长度: ",robot.path.size());
            commandManager.addRobotCommand(command);
        }
    }
    
}

void GameManager::shipControl(){

    // 执行船调度
    this->shipScheduler->scheduleShips(this->gameMap, this->ships, this->berths, this->goods, this->robots);
    // 对需要移动的船执行shipControl
    // todo 修改为海洋单行路
    shipController->runController(gameMap, this->singleLaneManager);
    // 执行指令
    for (Ship& ship : ships) {
        // 恢复状态
        if (ship.state == 1) continue;
        // 靠泊
        if (ship.shipStatus == ShipStatus::LOADING && ship.state == 0){
            commandManager.addShipCommand(ship.berth());
        }
        // 移动指令
        else if(! ship.path.empty()){
            string command = ship.movetoNextPosture();
            commandManager.addShipCommand(command);
        }
    }
}


void GameManager::update()
{   

    auto start = std::chrono::steady_clock::now();
    
    bool robotDebugOutput = false;
    bool shipDebugOutput = true;

    robotControl();


    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGI("robotControl时长:",duration.count(),"ms");

    if(shipDebugOutput){LOGI("船只开始调度");};
    auto ship_start = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<ShipID, ShipActionSpace::ShipAction>> shipActions = this->shipScheduler->scheduleShips(this->gameMap, this->ships, this->berths, this->goods, this->robots);
    auto ship_end = std::chrono::high_resolution_clock::now();
    LOGI("调度船只时长:",std::chrono::duration_cast<std::chrono::milliseconds>(ship_end - ship_start).count(),"ms");

    LOGI("命令个数：",shipActions.size());
    // CommandManager.shipCommands
    // for (int i = 0; i < shipActions.size(); i++)
    // {
    //     int ship_id = shipActions[i].first;
    //     ShipActionSpace::ShipAction ship_action = shipActions[i].second;
    //     LOGI("船:", ship_id);
    //     LOGI("船的命令类型:", ship_action.type);
    //     LOGI("船的执行id：",ship_action.targetId);
    //     // 去虚拟点
    //     if (ship_action.type == ShipActionSpace::ShipActionType::MOVE_TO_DELIVERY)
    //     {
    //         // LOGI(ship_id,"前往虚拟点");
    //         commandManager.addShipCommand(ships[ship_id].go(berths[ship_action.targetId].time));
    //     }
    //     // 去泊位
    //     if (ship_action.type == ShipActionSpace::ShipActionType::MOVE_TO_BERTH)
    //     {
    //         // LOGI(ship_id,"分配去泊位",ship_action.targetId);
    //         commandManager.addShipCommand(ships[ship_id].moveToBerth(ship_action.targetId));
    //     }
    // }
    LOGI("船命令执行完毕");

    if(currentFrame>=14000 && currentFrame <= 14005){
        LOGI("skipFrame: ", skipFrame, ", totalGetGoodsValue: ", totalGetGoodsValue);
        LOGI("berthDistrubtGoodNumCount: ",Log::printVector(berthDistrubtGoodNumCount));
        LOGI("berthDistrubtGoodValueCount: ",Log::printVector(berthDistrubtGoodValueCount));
    }
    if(currentFrame >=14990 && currentFrame <= 15000){
        LOGI("游戏结束，泊位剩余情况：");
        for(auto &berth : berths){
            // if(debug){LOGI("计算泊位收益-----");berth.info();}
            berth.totalValue = 0;
            for(auto & good : berth.reached_goods) berth.totalValue += good.value;
        }
        for(auto & berth : berths){
            berth.info();
        }
        LOGI("游戏结束，船只剩余情况：");
        for(auto &ship : ships){
            ship.info();
        }
    }
}

void GameManager::updateSingleLaneLocks()
{
    for (const Robot &robot : robots) {
        int currentSingleLaneID = singleLaneManager.getSingleLaneId(robot.pos);
        int nextFrameSingleLaneID1 = singleLaneManager.getSingleLaneId(robot.nextPos);

        // 即将进入单行道
        if (nextFrameSingleLaneID1 >= 1 && currentSingleLaneID == 0) {
            // LOGI("加锁 laneID: ", nextFrameSingleLaneID1, " robot: ", robot);
            singleLaneManager.lock(nextFrameSingleLaneID1, robot.nextPos);
        }
        // 即将离开单行道
        else if (nextFrameSingleLaneID1 == 0 && currentSingleLaneID >= 1) {
            // LOGI("释放锁 laneID: ", currentSingleLaneID, " robot: ", robot);
            singleLaneManager.unlock(currentSingleLaneID, robot.pos);
        }
    }
}

void GameManager::outputCommands()
{

    commandManager.outputCommands();
    commandManager.clearCommands();
}

void GameManager::onBerthStatusChanged(int berthId, bool isEnabled)
{
    LOGI("Berth 状态改变, ID: ", berthId, ", isEnabled: ", isEnabled);
    // 泊位被启用，遍历所有货物，往 distsToBerths 中添加可达泊位的距离
    if (isEnabled)
    {
        for (Goods &g : goods)
        {
            int distance = this->gameMap.getDistanceToBerth(berthId, g.pos);
            if (distance < INT_MAX)
            {
                std::pair<BerthID, int> newItem(berthId, distance);
                auto it = std::lower_bound(g.distsToBerths.begin(),
                                           g.distsToBerths.end(),
                                           newItem,
                                           [](const std::pair<BerthID, int> &a,
                                              const std::pair<BerthID, int> &b)
                                           {
                                               return a.second < b.second;
                                           });
                g.distsToBerths.insert(it, newItem);
            }
        }
    }
    // 泊位被禁用，遍历所有货物，删除 distsToBerths 中含有 berthID 的元素
    else
    {
        for (Goods &g : goods)
        {
            for (auto it = g.distsToBerths.begin(); it != g.distsToBerths.end();)
            {
                if (it->first == berthId)
                {
                    it = g.distsToBerths.erase(it);
                    break; // 退出循环
                }
                else
                {
                    ++it; // 只有在不删除元素时才向前移动迭代器
                }
            }
        }
    }
}


StageType GameManager::nowStateType()
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
            minVelocity = std::min(minVelocity, berth.velocity), maxTime = std::max(maxTime, berth.timeToDelivery());

        finalFrame = 15000 - maxTime * 3 - static_cast<int>(maxCapacity / minVelocity) * 2 - 500;
    }
    // LOGI("终局帧数：",finalFrame);
    // LOGI("当前帧数：",currentFrame);
    if (currentFrame < finalFrame)
    {
        return StageType::BEGIN;
    }
    else
    {
        #ifdef DEBUG
        LOGI("进入终局调度，终局帧数：",finalFrame,",当前帧数：",currentFrame);
        #endif
        return StageType::FINAL;
    }
}
