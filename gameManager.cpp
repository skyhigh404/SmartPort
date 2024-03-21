#include "gameManager.h"
#include <string>
#include <iostream>
#include "log.h"
#include "pathFinder.h"
#include <chrono>

using namespace std;
int Goods::number = 0;
int Berth::totalLoadGoodnum = 0;
int Berth::maxLoadGoodNum = 0;
int Berth::deliverGoodNum = 0;
std::vector<bool> Berth::available_berths = std::vector<bool>(BERTHNUMS,true);  //  泊位是否可获取，用于终局调度
int CURRENT_FRAME = 0;  //当前帧数
int canUnload(Berth& berth, Point2d pos) {
    int x = pos.x-berth.pos.x, y=pos.y-berth.pos.y;
    if (x<0 || x>3 || y<0 || y>3) {LOGI("越界",pos,' ',berth.pos);return 0;}
    // if (berth.storageSlots[x][y]== -1) {LOGI("可放貨");return 1;}
    else return 1;
}

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
    // for (int i = 0; i < SHIPNUMS; ++i)
    // {
    //     LOGI("Ship ", this->ships[i].id," capacity: ", this->ships[i].capacity);
    // }

    // 初始化数据读取完成
    // 让地图实时跟踪机器人位置（需要测试是否正常跟踪）
    for (Robot &robot : this->robots)
        this->gameMap.robotPosition.push_back(robot.pos);

    // 计算地图上每个点到泊位的距离
    auto bfs_start = std::chrono::high_resolution_clock::now();
    for (const auto &berth : this->berths)
    {
        vector<Point2d> positions;
        // 泊位大小 4x4
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                positions.push_back(berth.pos + Point2d(i, j));

        this->gameMap.computeDistancesToBerthViaBFS(berth.id, positions);
    }
    auto bfs_end = std::chrono::high_resolution_clock::now();
    LOGI("bfs初始化时长:",std::chrono::duration_cast<std::chrono::milliseconds>(bfs_end - bfs_start).count(),"ms");


    // 判断机器人是否位于死点
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
            {robot.status = DEATH;LOGI("死機器人:",robot.id);}
    }

    // 初始化 RobotController
    this->robotController = std::make_shared<RobotController>(this->robots);
    auto start = std::chrono::steady_clock::now();
    this->singleLaneManager.init(gameMap);
    auto end = std::chrono::steady_clock::now();

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
    
    int findTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    LOGI("单行路初始化处理时间：",findTime,"ms");
    // 打印单行路
    LOGI("单行路数量：",this->singleLaneManager.singleLanes.size());
    // LOGI(Map::drawMap(this->singleLaneManager.singleLaneMap,3));
    // LOGI("输出单行路锁信息");
    // for (const auto& pair : this->singleLaneManager.singleLaneLocks) {
    //     int laneId = pair.first;
    //     const SingleLaneLock& lock = pair.second;
    //     LOGI("ID: ",laneId, " startPos: ", lock.startPos, ", endPos: ", lock.endPos);
    //     // LOGI("-----------------------------------------单行路路径:");
    //     // for(auto &point : this->singleLaneManager.singleLanes[laneId]){
    //     //     LOGI(point);
    //     // }
    // }
    // LOGI(this->gameMap.drawMap(nullptr, nullptr, nullptr, nullptr, nullptr));
    // LOGI("Log berth 0 BFS map.");
    // LOGI(Map::drawMap(this->gameMap.berthDistanceMap[9],12));
    // exit(0);
}

void GameManager::processFrameData()
{
    LOGI("processFrameData.");
    int newItemCount;
    int goodsX, goodsY, value;
    int carrying, robotX, robotY, robotState;
    int shipState, berthId;
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
    LOGI("====================================================新的一帧=====================================================");
    // 货物生命周期维护
    for (auto& good : goods){
        // todo 边界条件
        if(good.TTL != INT_MAX && good.TTL >= 0){
            // LOGI(good.id,",initFrame:",good.initFrame,",currentFrame:",currentFrame,",TTL:",good.TTL);
            good.TTL = std::max(1000-(currentFrame - good.initFrame),-1);
        }
    }
    // 读取新增货物
    cin >> newItemCount;
    while (newItemCount--)
    {
        cin >> goodsX >> goodsY >> value;
        this->goods.emplace_back(Point2d(goodsX, goodsY), value, currentFrame);
    }
    // 读取机器人状态
    bool flag = false;
    for (int i = 0; i < ROBOTNUMS; ++i)
    {
        cin >> carrying >> robotX >> robotY >> robotState;
        this->robots[i].carryingItem = carrying;
        this->robots[i].pos.x = robotX;
        this->robots[i].pos.y = robotY;
        this->robots[i].state = robotState;

        if(robotState==0){
            LOGE("Robot ", i," 发生碰撞, pos: ", robots[i].pos);
            flag = true;
        }

        // 暂时处理程序认为机器人放下了货物并且分配了下一个货物的id，但是判题器认为机器人还拿着上一个货物的情况
        if (this->robots[i].carryingItem == 0)
        {
            // 强行初始化机器人状态
            this->robots[i].carryingItemId = -1;
        }
        // 检查是否要 pop path
        this->robots[i].updatePath();

        // 暂时处理程序认为机器人放下了货物并且分配了下一个货物的id，但是判题器认为机器人还拿着上一个货物的情况
        if(this->robots[i].carryingItem == 0){
            // 强行初始化机器人状态
            this->robots[i].carryingItemId = -1;
        }
    }
    // if(flag)
        // exit(0);

    // 读取船舶状态
    for (int i = 0; i < SHIPNUMS; ++i)
    {
        cin >> shipState >> berthId;
        this->ships[i].state = shipState;
        this->ships[i].berthId = berthId;
        if(ships[i].state == 0 && ships[i].berthId != -1){
            LOGW("船只状态：",ships[i].state,",船只泊位：",ships[i].berthId);
        }
    }
    // 确认已接收完本帧的所有数据
    string ok;
    cin >> ok;

    // 初始化泊位货物状态
    for(auto &berth : berths){
        berth.unreached_goods = std::vector<Goods>();
        // berth.reached_goods = std::vector<Goods>();
    }

    // for (int i = 0; i < ROBOTNUMS; ++i)
    // {
    //     LOGI("Robot: ", this->robots[i].id, " position: ", this->robots[i].pos, " state: ", this->robots[i].state);
    // }
}

void GameManager::robotControl()
{
    bool robotDebugOutput = false;
    // 机器人状态更新
    for (Robot& robot:robots) {
        if (robot.status==DEATH) continue;
        // 机器人眩晕
        if (robot.status == DIZZY || robot.state == 0) {
            // 还在眩晕状态
            robot.status = DIZZY;
            if (robot.state == 0) continue;

            // 从眩晕状态恢复
            if (robotDebugOutput) LOGI("从眩晕状态恢复");
            if (robot.carryingItem == 0) {
                robot.status = MOVING_TO_GOODS;
                robot.path = Path();
                robot.targetid = -1;
                robot.destination = Point2d(-1,-1);
                robot.carryingItemId = -1;
            }
            else {
                robot.status = MOVING_TO_BERTH;
                robot.path = Path();
                robot.targetid = -1;
                robot.destination = Point2d(-1,-1);
            }
        }

        // 机器人状态更新
        if (robot.carryingItem==0) robot.status = MOVING_TO_GOODS;
        else robot.status = MOVING_TO_BERTH;
    }

    // 对所有可能的机器人执行取货或放货指令，更新状态
    for (Robot& robot : robots) {
        if (robot.status==DEATH) continue;
        if (robot.status==MOVING_TO_GOODS && robot.targetid!=-1 && robot.pos == goods[robot.targetid].pos) {
            if (goods[robot.targetid].TTL>0) {
                                commandManager.addRobotCommand(robot.get());
                robot.carryingItem = 1;
                robot.carryingItemId = robot.targetid;
                robot.status = MOVING_TO_BERTH;
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
        else if(robot.status==MOVING_TO_BERTH && robot.targetid!=-1 && robot.pos == robot.destination) {
            Berth &berth = berths[robot.targetid];
            // LOGI(robot);
            if (canUnload(berth, robot.pos)) {
                LOGI("機器人",robot.id,"放貨 ");
                commandManager.addRobotCommand(robot.pull());
                int x = robot.pos.x-berth.pos.x, y=robot.pos.y-berth.pos.y;
                // berth.storageSlots[x][y] = goods[robot.carryingItemId].id;
                if(currentFrame < 15000 - berth.time){
                    Berth::maxLoadGoodNum += 1;
                    totalGetGoodsValue += goods[robot.carryingItemId].value;
                    berth.reached_goods.push_back(goods[robot.carryingItemId]);
                    goods[robot.carryingItemId].status = 3;
                }
                goods[robot.carryingItemId].status = 3;
                robot.status = MOVING_TO_GOODS;
                robot.carryingItem = 0;
                robot.carryingItemId = -1;
                robot.targetid = -1;
            }
            else {
                robot.targetid = -1;
            }
        }
    }
    // LOGI("機器人取放貨完畢");

    if (this->RobotScheduler->getSchedulerType()==FINAL && this->RobotScheduler->enterFinal==false) {
        LOGI("機器人調度進入終局");
        for (Robot& robot:robots) {
            if ( (robot.status==MOVING_TO_BERTH && Berth::available_berths[robot.targetid]==false) || (robot.status==MOVING_TO_GOODS && Berth::available_berths[this->RobotScheduler->bestBerthIndex[robot.targetid][0]==false])) {
                robot.targetid = -1;
                robot.destination = Point2d(-1,-1);
                robot.path = Path();
            }
        }
        this->RobotScheduler->bestBerthIndex.clear();
        this->RobotScheduler->enterFinal = true;
    }

    auto start = std::chrono::steady_clock::now();
    // 对所有需要调度的机器人进行调度
    for (Robot& robot : robots) {
        if (robot.status==DEATH) continue;
        if ((robot.status==MOVING_TO_GOODS && robot.targetid==-1) || (robot.status==MOVING_TO_BERTH && robot.targetid==-1)) {
            Action action = this->RobotScheduler->scheduleRobot(robot, gameMap, goods, berths, robotDebugOutput);
            if (action.type==FAIL) {
                LOGI("機器人",robot.id,"調度失敗");
                robot.targetid = -1;
                robot.destination = Point2d(-1,-1);
                continue;
            }
            robot.targetid = action.targetId;
            robot.destination = action.desination;
        }
    }

    // // 分配货物
    // vector<Robot> needSchedule;
    // for (Robot& robot : robots) {
    //     if (robot.status==DEATH) continue;
    //     if (needSchedule.size()>=3) break;
    //     if ((robot.status==MOVING_TO_GOODS && robot.targetid==-1)) {
    //         needSchedule.push_back(robot);
    //     }
    // }
    // vector<Goods> availableGoods;
    // for (Goods& good : goods) {
    //     if (good.status==0) availableGoods.push_back(good);
    // }
    // vector<int> array(needSchedule.size(), -1); int idx=0;
    // this->RobotScheduler->calCostAndBestBerthIndes(gameMap, goods, berths);
    // this->RobotScheduler->LPscheduleRobots(needSchedule, gameMap, availableGoods, berths, array, idx);
    // vector<int> scheduleResult = this->RobotScheduler->getResult();
    // for (int i=0;i<scheduleResult.size();i++) {
    //     // LOGI(scheduleResult[i]);
    //     int index = scheduleResult[i];
    //     Robot& robot = robots[needSchedule[i].id];
    //     if (index==-1) {
    //         LOGI("機器人",robot.id,"調度失敗");
    //         robot.targetid = -1;
    //         robot.destination = Point2d(-1,-1);
    //         continue;
    //     }
    //     robot.targetid = availableGoods[index].id;
    //     robot.destination = availableGoods[index].pos;
    //     goods[availableGoods[index].id].status = 1;
    //     LOGI("机器人分配货物：",robot.id,' ',robot.targetid,' ',robot.destination);
    // }
    // // 分配泊位
    // for (Robot& robot:robots) {
    //     // LOGI(robot);
    //     if (robot.status==DEATH) continue;
    //     Action action = this->RobotScheduler->scheduleRobot(robot, gameMap, goods,berths, false);
    //     if (action.type!=FAIL) {
    //         robot.targetid = action.targetId;
    //         robot.destination = action.desination;
    //     }
    // }
    auto end = std::chrono::steady_clock::now();
    LOGI("scheduleRobots时间: ",std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()," ms");
    // LOGI("機器人調度完畢");

    // 执行动作
    robotController->runController(gameMap, this->singleLaneManager);
    // LOGI("機器人尋路完畢");
    // 维护单行路的锁
    // updateSingleLaneLocks();
    
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


void GameManager::RobotControl()
{
    bool robotDebugOutput = false;
    AStarPathfinder pathfinder;
    for (int i=0;i<robots.size();i++) {
        Robot& robot = robots[i];
        if (robot.status == DEATH) continue;
        if (robot.carryingItem == 1) robot.status = MOVING_TO_BERTH;

        if (robot.status == DIZZY || robot.state == 0) {
            robot.status = DIZZY;
            // 还在眩晕状态
            if (robot.state == 0) continue;

            // 从眩晕状态恢复
            if (robotDebugOutput) LOGI("从眩晕状态恢复");

            if (robot.carryingItem == 0) {
                robot.status = IDLE;
                robot.path = Path();
                robot.destination = Point2d(0,0);
                robot.targetid = -1;
                robot.carryingItemId = -1;
            }
            else {
                robot.status = MOVING_TO_BERTH;
                robot.path = Path();
                robot.targetid = -1;
            }
        }

        if (robot.status == IDLE) {
            Action action = this->RobotScheduler->scheduleRobot(robots[i], gameMap, goods, berths, robotDebugOutput);
            if (action.type==FAIL) continue;
            std::variant<Path, PathfindingFailureReason> path = pathfinder.findPath(robots[i].pos, action.desination, gameMap);
            if (std::holds_alternative<Path>(path)) {
                if (robotDebugOutput) LOGI(i, "寻路成功");
                Path temp_path = std::get<Path>(path);
                robot.path = temp_path;
                robot.status = MOVING_TO_GOODS;
                robot.targetid = action.targetId;
                robot.destination = action.desination;
                // if(robotDebugOutput && i == 1){LOGI("分配路径：");LOGI(robots[i]);LOGI("目的地位置：",action.desination);LOGI("货物位置：",goods[robots[i].targetid].pos);LOGI("货物TTL：",goods[robots[i].targetid].TTL);}
            }
            else {
                if (robotDebugOutput) LOGI(i, "寻路失败");
                // 重置窗台
                robots[i].path = Path();
                robot.status = IDLE;
                robot.targetid = -1;
                // robot.destination = Point2d(-1,-1);
            }
        }

        if (robot.status == MOVING_TO_GOODS) {
            if (!robot.path.empty()) {
                const std::string temp = robot.moveWithPath();
                if (robotDebugOutput) LOGI(i, "向货物移动中:",temp, robot.path.size());
                commandManager.addRobotCommand(temp);
                continue; // 将移动和取货分开，防止碰撞导致的取货失败，但是将多消耗一帧
            }

            // // 看看有没有更优的货物
            // Action action = this->scheduler->scheduleRobot(robots[i], gameMap, goods, berths, robotDebugOutput);
            // if (action.type!=FAIL) {
            //     if (action.targetId != robot.targetid) {
            //         std::variant<Path, PathfindingFailureReason> path = pathfinder.findPath(robot.pos, action.desination, gameMap);
            //         if (std::holds_alternative<Path>(path)) {
            //             // if (robotDebugOutput) LOGI(i, "寻路成功");
            //             Path temp_path = std::get<Path>(path);
            //             robot.path = temp_path;
            //             robot.targetid = action.targetId;
            //             robot.destination = action.desination;
            //             // if(robotDebugOutput && i == 1){LOGI("分配路径：");LOGI(robots[i]);LOGI("目的地位置：",action.desination);LOGI("货物位置：",goods[robots[i].targetid].pos);LOGI("货物TTL：",goods[robots[i].targetid].TTL);}
            //         }
            //         else {
            //             if (robotDebugOutput) LOGI(i, "寻路失败");
            //             // 重置窗台
            //             robots[i].path = Path();
            //             robot.status = IDLE;
            //             robot.targetid = -1;
            //             // robot.destination = Point2d(-1,-1);
            //         }
            //     }
            // }

            // 如果未达到目标货物，但路径为空
            if (robot.pos != robot.destination && robot.path.empty()) {
                std::variant<Path, PathfindingFailureReason> path = pathfinder.findPath(robot.pos, robot.destination, gameMap);
                if (std::holds_alternative<Path>(path)) {
                    Path temp_path = std::get<Path>(path);
                    robot.path = temp_path;
                }
                else {
                    LOGI("已分配货物，但路径为空，且寻路失败.",robot.pos,',',robot.destination);
                    robot.status = IDLE;
                    robot.targetid = -1;
                }
            }
            // 到达货物位置
            Goods& good = goods[robot.targetid];
            if (robot.pos == good.pos) {
                // 可以取货
                if (good.TTL>0) {
                    commandManager.addRobotCommand(robot.get());
                    robots[i].carryingItem = 1;
                    robots[i].carryingItemId = robots[i].targetid;
                    robot.status = MOVING_TO_BERTH;
                    robot.targetid = -1;

                    good.TTL = INT_MAX;
                }
                // 货物过期
                else {
                    robot.status = IDLE;
                    robot.targetid = -1;
                    continue;
                }
            }
            continue;
        }

        if (robot.status == MOVING_TO_BERTH) {
            // 分配泊位
            if (robot.targetid == -1) {
                Action action = this->RobotScheduler->scheduleRobot(robot, gameMap, goods, berths, robotDebugOutput);
                std::variant<Path, PathfindingFailureReason> path = pathfinder.findPath(robot.pos, action.desination, gameMap);
                if (std::holds_alternative<Path>(path)) {
                    Path temp_path = std::get<Path>(path);
                    robot.path = temp_path;
                    robot.status = MOVING_TO_BERTH;
                    robot.targetid = action.targetId;
                    robot.destination = action.desination;
                }
                else {
                    robot.targetid = -1;
                }
                continue;
            }

            // 如果未到达目标泊位，但是路径为空
            if (robot.pos != robot.destination && robot.path.empty()) {
                std::variant<Path, PathfindingFailureReason> path = pathfinder.findPath(robot.pos, robot.destination, gameMap);
                if (std::holds_alternative<Path>(path)) {
                    Path temp_path = std::get<Path>(path);
                    robot.path = temp_path;
                }
                else {
                    // LOGI("已分配泊位，但路径为空，且寻路失败.",robot.pos,',',robot.destination);
                    robot.targetid = -1;
                }
            }
            
            const std::string temp = robot.moveWithPath();
            if (robotDebugOutput) LOGI(i, "向泊位移动中:",temp);
            commandManager.addRobotCommand(temp);

            // 放货
            Berth &berth = berths[robot.targetid];
            LOGI(robot);
            // berth.info(); //这个函数有bug，
            if (robot.pos == robot.destination) {
                // 货物可放货 todo
                // LOGI("放貨",robot,"dest",robot.destination);
                if (canUnload(berth, robot.pos)) {
                    // LOGI("輸出");
                    commandManager.addRobotCommand(robots[i].pull());
                    // 货物 泊位 状态更新 todo
                    int x = robot.pos.x-berth.pos.x, y=robot.pos.y-berth.pos.y;
                    // berth.storageSlots[x][y] = goods[robot.carryingItemId].id;
                    berth.reached_goods.push_back(goods[robot.carryingItemId]);


                    goods[robot.carryingItemId].status = 3;
                    // goods[robot.carryingItemId].TTL = ;

                    robot.status = IDLE;
                    robot.carryingItem = 0;
                    robot.carryingItemId = -1;
                    robot.targetid = -1;
                    Berth::maxLoadGoodNum += 1;
                }
                // 不可放货，重新分配放货位置
                else {
                    // LOGI(robot);
                    // berth.info();
                    robot.targetid = -1;
                }
            }
        }

    }
}



void GameManager::update()
{   
    auto start = std::chrono::steady_clock::now();
    
    bool robotDebugOutput = false;
    bool shipDebugOutput = true;

    // robots[3].findPath(gameMap,Point2d(133,99));
    // robots[6].findPath(gameMap,Point2d(142,112));
    // commandManager.addRobotCommand(robots[3].moveWithPath());
    // commandManager.addRobotCommand(robots[6].moveWithPath());
    // RobotControl();
    robotControl();

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOGI("robotControl时长:",duration.count(),"ms");

    if(shipDebugOutput){LOGI("船只开始调度");};
    auto ship_start = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, Action>> ShipActions = this->ShipScheduler->scheduleShips(ships, berths, goods, robots,this->RobotScheduler->bestBerthIndex,this->gameMap,this->currentFrame, shipDebugOutput);
    auto ship_end = std::chrono::high_resolution_clock::now();
    LOGI("调度船只时长:",std::chrono::duration_cast<std::chrono::milliseconds>(ship_end - ship_start).count(),"ms");

    // CommandManager.shipCommands
    for (int i = 0; i < ShipActions.size(); i++)
    {
        int ship_id = ShipActions[i].first;
        Action ship_action = ShipActions[i].second;
        // 去虚拟点
        if (ship_action.type == DEPART_BERTH)
        {
            // LOGI(ship_id,"前往虚拟点");
            commandManager.addShipCommand(ships[ship_id].go());
        }
        // 去泊位
        if (ship_action.type == MOVE_TO_BERTH)
        {
            // LOGI(ship_id,"分配去泊位",ship_action.targetId);
            commandManager.addShipCommand(ships[ship_id].moveToBerth(ship_action.targetId));
        }
    }

    if(currentFrame>=14000 && currentFrame <= 14005){
        LOGI("skipFrame: ", skipFrame, ", totalGetGoodsValue: ", totalGetGoodsValue);
    }
    if(currentFrame >=14990 && currentFrame <= 14999){
        LOGI("游戏结束，泊位剩余情况：");
        for(auto &berth : berths){
            // if(debug){LOGI("计算泊位收益-----");berth.info();}
            berth.totalValue = 0;
            for(auto & good : berth.reached_goods) berth.totalValue += good.value;
        }
        for(auto & berth : berths){
            berth.info();
        }
        LOGI("游戏结束，泊位剩余情况：");
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
            LOGI("加锁 laneID: ", nextFrameSingleLaneID1, " robot: ", robot);
            singleLaneManager.lock(nextFrameSingleLaneID1, robot.nextPos);
        }
        // 即将离开单行道
        else if (nextFrameSingleLaneID1 == 0 && currentSingleLaneID >= 1) {
            LOGI("释放锁 laneID: ", currentSingleLaneID, " robot: ", robot);
            singleLaneManager.unlock(currentSingleLaneID, robot.pos);
        }
    }
}

void GameManager::outputCommands()
{

    commandManager.outputCommands();
    commandManager.clearCommands();
}
