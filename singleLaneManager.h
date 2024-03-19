#pragma once

#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include "utils.h"
#include "map.h"
#include "robot.h"

// //  单行路
// struct SingleLane {
// public:
//     Point2d start; // 单行路起点坐标
//     Point2d end;   // 单行路终点坐标
//     bool hasRobot; // 标志位，表示当前是否有机器人在单行路上

//     SingleLane(Point2d start, Point2d end, bool hasRobot = false) : start(start), end(end), hasRobot(hasRobot) {}
// };

// // 只能找到单段单行路
// class SingleLaneManager {
// private:
//     std::vector<SingleLane> singleLanes; // 存储所有单行路的集合
//     std::unordered_map<int, bool> hasRobots;    //每个死锁路上是否有机器人
//     std::vector<std::vector<int>> singleLaneMap;    //单行路地图，如果对应取值为-1则不是单行路

// public:
//     SingleLaneManager(){
        
//     }

//     SingleLaneManager(Map &map){
//         findSingleLanes(map);
//     }

//     // 初始化地图中的单行路
//     void findSingleLanes( Map& map_temp) {
//         Map map = Map(map_temp);
//         std::vector<std::vector<int>> flagMap(map.rows,std::vector<int>(map.cols,0));   //标记是否访问过
//         singleLanes.clear(); // 清除之前的结果
//         auto ship_start = std::chrono::high_resolution_clock::now();
//         LOGI("开始寻找单行路");
//         for (int x = 0; x < map.rows; x++) {
//             for (int y = 0; y < map.cols; y++) {
//                 // LOGI("当前坐标：",x,",",y);
//                 // LOGI("数量：",singleLanes.size());
                
//                 // if (flagMap[x][y] == 0 && map_temp.grid[x][y] == MapItemSpace::MapItem::SPACE) {
//                 if (flagMap[x][y] == 0 && (isSinglePos(map,Point2d(x,y),0)) ) {
//                     // 检查向右的单行路
//                     int nextX = x ;
//                     while (nextX < map.rows && isSinglePos(map,Point2d(nextX,y),0) && flagMap[nextX][y] == 0) {
//                         flagMap[nextX][y] = 1;
//                         nextX++;
//                     }
//                     // if (nextX - x > 1) { // 发现单行路
//                     //     // singleLanes.push_back(SingleLane(Point2d(x, y), Point2d(std::min(nextX,map.rows -1), y)));
//                     //     singleLanes.push_back(SingleLane(Point2d(std::max(x - 1,0), y), Point2d(std::min(nextX,map.rows -1), y)));
//                     // }
//                     int startx,endx;
//                     if (x - 1 >=0 && map.grid[x-1][y] == MapItemSpace::MapItem::SPACE) startx = x-1;
//                     else startx = x;
//                     if (nextX < map.rows && map.grid[nextX][y] == MapItemSpace::MapItem::SPACE)  endx = nextX;
//                     else endx = nextX -1;
//                     // todo 可能有顶上是泊位的特殊情况
//                     // if(startx != endx && !(startx == x && endx == nextX -1)) singleLanes.push_back(SingleLane(Point2d(startx, y), Point2d(endx, y)));
//                     if(startx != endx && !isDeathLane(map_temp,Point2d(startx, y), Point2d(endx, y))) singleLanes.push_back(SingleLane(Point2d(startx, y), Point2d(endx, y)));
//                 }
//                 if (flagMap[x][y] == 0 && (isSinglePos(map,Point2d(x,y),0)) || (isSinglePos(map,Point2d(x,y),1))) {
//                     // 检查向下的单行路
//                     int nextY = y ;
//                     while (nextY < map.cols &&map.grid[x][nextY] == MapItemSpace::MapItem::SPACE
//                     && isSinglePos(map,Point2d(x,nextY),1) && flagMap[x][nextY] == 0) {
//                         flagMap[x][nextY] = 1;
//                         ++nextY;
//                     }
//                     // if (nextY - y > 1) { // 发现单行路
//                     //     singleLanes.push_back(SingleLane(Point2d(x, std::max(y - 1,0)), Point2d(x, std::min(nextY,map.cols - 1))));
//                     // }
//                     int starty, endy;
//                     if(y - 1 >=0 && map.grid[x][y-1] == MapItemSpace::MapItem::SPACE) starty = y-1;
//                     else starty = y;
//                     if(nextY < map.cols && map.grid[x][nextY] == MapItemSpace::MapItem::SPACE) endy = nextY;
//                     else endy = nextY -1;
//                     // if(starty != endy && !(starty == y && endy == nextY -1)) singleLanes.push_back(SingleLane(Point2d(x,starty),Point2d(x,endy)));
//                     if(starty != endy && !isDeathLane(map_temp,Point2d(x,starty),Point2d(x,endy))) singleLanes.push_back(SingleLane(Point2d(x,starty),Point2d(x,endy)));
//                 }
//             }
//         }
//         auto ship_end = std::chrono::high_resolution_clock::now();
//         LOGI("寻路时长:",std::chrono::duration_cast<std::chrono::milliseconds>(ship_end - ship_start).count(),"ms");
//         LOGI("寻找完毕！！");
        
//         // 对单行路进行排序
//         // std::sort(singleLanes.begin(), singleLanes.end(), [&](SingleLane a, SingleLane b) {
//         LOGI("初始化单行路地图");
//         ship_start = std::chrono::high_resolution_clock::now();
//         singleLaneMap = std::vector<std::vector<int>>(map.rows, std::vector<int>(map.cols, -1));
//         int single_index = 0;   //单行路序号
//         for(auto singleLane : singleLanes){
//             // 单行路联通
//             int now_index = single_index;
//             hasRobots[now_index] = false;
//             // 两头都联通
//             if (singleLaneMap[singleLane.start.x][singleLane.start.y] != -1 && singleLaneMap[singleLane.end.x][singleLane.end.y] != -1){
//                 // 随机选取一个
//             }
//             single_index++;
//         }
//         ship_end = std::chrono::high_resolution_clock::now();
//         LOGI("初始化地图时长:",std::chrono::duration_cast<std::chrono::milliseconds>(ship_end - ship_start).count(),"ms");
        
//     }

//     // 判断是否思路
//     bool isDeathLane(Map &map,Point2d start,Point2d end){
//         bool startDeath = false,endDeath = false;
//         // todo 除了Space外都是障碍
//         if(start.x == end.x){   //纵向 
//             if(start.y -1  < 0 || (map.grid[start.x][start.y-1] != MapItemSpace::MapItem::SPACE )){
//                 startDeath = true;
//                 // LOGI("起点死路",start);
//             }else{
//                 // LOGI((map.grid[start.x][start.y-1] == MapItemSpace::MapItem::SPACE));
//             }

//             if(start.y + 1 >= map.cols || (map.grid[start.x][start.y+1] != MapItemSpace::MapItem::SPACE )){
//                 endDeath = true;
//                 // LOGI("终点死路",end);
//             }else{
//                 // LOGI((map.grid[start.x][start.y+1] == MapItemSpace::MapItem::SPACE ));
//             }
//         }else{  //横向
//             if(start.x - 1 < 0 || (map.grid[start.x - 1][start.y] != MapItemSpace::MapItem::SPACE )){
//                 startDeath = true;
//                 // LOGI("起点死路",start);
//             }else{
//                 // LOGI(map.grid[start.x - 1][start.y] == MapItemSpace::MapItem::SPACE );
//             }

//             if(start.x + 1 >= map.rows || (map.grid[start.x + 1][start.y] != MapItemSpace::MapItem::SPACE )){
//                 endDeath = true;
//                 // LOGI(map.grid[start.x + 1][start.y] == MapItemSpace::MapItem::SPACE );
//             }else{
//                 // LOGI("终点死路",end);
//             }
//         }
//         return (startDeath && endDeath);
//     }

//     // 判断是否是单行格子
//     // flag 0 横向;1 纵向
//     bool isSinglePos(Map &map,Point2d pos,int flag){
//         Point2d left,right;
//         switch (flag)
//         {
//         case 0:
//             left = Point2d(pos.x,pos.y - 1);
//             right = Point2d(pos.x,pos.y + 1);
//             break;

//         case 1:
//             left = Point2d(pos.x - 1,pos.y);
//             right = Point2d(pos.x + 1,pos.y);
//             break;

//         default:
//             break;
//         }
//             // 传入位置是空的，其他x y是障碍
//             if(map.grid[pos.x][pos.y] == MapItemSpace::MapItem::SPACE
//             && (!map.inBounds(left) || (map.grid[left.x][left.y] != MapItemSpace::MapItem::SPACE && map.grid[left.x][left.y] != MapItemSpace::MapItem::BERTH))
//             && (!map.inBounds(right) || map.grid[right.x][right.y] != MapItemSpace::MapItem::SPACE && map.grid[left.x][left.y] != MapItemSpace::MapItem::BERTH)){
//                 // LOGI("left:",left,",right:",right);
//                 return true;
//         }
//         return false;
//     }

//     const std::vector<SingleLane>& getSingleLanes() const {
//         return singleLanes;
//     }

//     std::vector<Point2d> getSingleLanesVector() const {
//         std::vector<Point2d> res;
//         for(auto &singleLane : singleLanes){
//             res.push_back(singleLane.start);
//             res.push_back(singleLane.end);
//         }
//         return res;
//     }

//     // 判断机器人是否即将进入单行路，并根据hasRobot判断是否通行
//     bool canRobotEnter(const Robot& robot) {
//         Point2d robotPos = robot.pos;
//         for (const auto& lane : singleLanes) {
//             // 检查机器人当前位置是否在单行路的一个端点附近
//             if (robotPos.x == lane.start.x && robotPos.y == lane.start.y - 1 ||
//                 robotPos.x == lane.start.x && robotPos.y == lane.start.y + 1 ||
//                 robotPos.x == lane.end.x && robotPos.y == lane.end.y - 1 ||
//                 robotPos.x == lane.end.x && robotPos.y == lane.end.y + 1) {
//                 // 如果机器人即将进入单行路，检查该路是否已有机器人占用
//                 return !lane.hasRobot;
//             }
//         }
//         return true; // 如果不是单行路，或单行路没有被占用，返回true
//     }

//     // 更新地图中单行路的hasRobot
//     void updateSingleLanesWithRobots(const std::vector<Robot>& robots) {
//         // 首先，假设所有单行路上没有机器人
//         for (auto& lane : singleLanes) {
//             lane.hasRobot = false;
//         }

//         // 检查每个机器人，如果它在某条单行路上，更新该单行路的hasRobot状态
//         for (const auto& robot : robots) {
//             for (auto& lane : singleLanes) {
//                 // 检查机器人是否在该单行路上
//                 if (isRobotOnLane(robot.pos, lane)) {
//                     lane.hasRobot = true;
//                     break; // 一个机器人只能在一条单行路上，所以找到后就可以停止检查
//                 }
//             }
//         }
//     }

// private:
//     // 辅助函数：检查机器人是否在给定的单行路上
//     bool isRobotOnLane(const Point2d& robotPos, const SingleLane& lane) {
//         // 这里的实现需要根据单行路的具体定义，以下是一种可能的简化实现
//         // 假设单行路是直线，只检查机器人是否在单行路的起点和终点之间
//         if (lane.start.x == lane.end.x) { // 垂直单行路
//             return robotPos.x == lane.start.x && robotPos.y >= lane.start.y && robotPos.y <= lane.end.y;
//         } else if (lane.start.y == lane.end.y) { // 水平单行路
//             return robotPos.y == lane.start.y && robotPos.x >= lane.start.x && robotPos.x <= lane.end.x;
//         }
//         return false;
//     }
// };

enum VisitType
{
    VISITED,
    // FINAL_READY,
    UNVISITED
};

struct SingleLaneLock
{
    Point2d startPos;
    Point2d endPos;
    bool startLock;
    bool endLock;
    SingleLaneLock(Point2d start,Point2d end):startPos(start),endPos(end),startLock(false),endLock(false){}
};



class SingleLaneManger {
public:
    int rows, cols;
    std::vector<std::vector<int>> singleLaneMap;    //  单行路位置标记为单行路的id，其余位置标记为-1
    std::unordered_map<int,SingleLaneLock> singleLaneLocks;    // 维护每个单行路的锁，标记当前是否通行
    std::vector<std::vector<MapItemSpace::MapItem>> grid;    //原地图
    std::vector<std::vector<VisitType>> visited; // 访问过的位置
    std::unordered_map<int,std::vector<Point2d>> singleLanes;   //存储单行路的路径

    int nextSingleLaneId = 1;   // 单行路的路径

    SingleLaneManger(){}

    SingleLaneManger(const Map& map){
        this->rows = map.rows;
        this->cols = map.cols;
        this->grid = map.grid;
        singleLaneMap = std::vector<std::vector<int>>(map.rows,std::vector<int>(map.cols,-1));
        visited = std::vector<std::vector<VisitType>>(map.rows,std::vector<VisitType>(map.cols,VisitType::UNVISITED));

        // 找到所有单行路
        findAllSingleLanes();
    }

    void init(const Map& map){
        this->rows = map.rows;
        this->cols = map.cols;
        // 拷贝复制
        this->grid = std::vector<std::vector<MapItemSpace::MapItem>>(map.grid);
        singleLaneMap = std::vector<std::vector<int>>(map.rows,std::vector<int>(map.cols,-1));
        visited = std::vector<std::vector<VisitType>>(map.rows,std::vector<VisitType>(map.cols,VisitType::UNVISITED));
        // 初始化地图
        initMap();
        // 找到所有单行路
        findAllSingleLanes();
    }

    bool isValid(const Point2d& pos) { return pos.x >= 0 && pos.x < rows && pos.y >= 0 && pos.y < cols; }

    bool canPass(const Point2d& pos) {
        return grid[pos.x][pos.y] == MapItemSpace::MapItem::SPACE || grid[pos.x][pos.y] == MapItemSpace::MapItem::BERTH;
    }

    // 判断是否是拐角
    bool isCorner(Point2d &pos){
        // bool left = (!isValid(Point2d(pos.x-1,pos.y)) || !canPass(Point2d(pos.x-1,pos.y)));
        // bool right = (!isValid(Point2d(pos.x+1,pos.y)) || !canPass(Point2d(pos.x+1,pos.y)));
        // bool up = (!isValid(Point2d(pos.x,pos.y+1)) || !canPass(Point2d(pos.x-1,pos.y+1)));
        // bool down = (!isValid(Point2d(pos.x,pos.y-1)) || !canPass(Point2d(pos.x,pos.y-1)));
        // return (left || right )&&(up || down);
        int left = 0,up = 0;
        if(!isValid({pos.x-1,pos.y})) left += -1;
        else left += singleLaneMap[pos.x-1][pos.y];
        if(!isValid({pos.x+1,pos.y})) left += -1;
        else left += singleLaneMap[pos.x+1][pos.y];

        if(!isValid({pos.x,pos.y-1})) up += -1;
        else up += singleLaneMap[pos.x][pos.y-1];
        if(!isValid({pos.x,pos.y+1})) up += -1;
        else up +=singleLaneMap[pos.x][pos.y+1];
        // int left = singleLaneMap[pos.x-1][pos.y] + singleLaneMap[pos.x+1][pos.y];
        // int up = singleLaneMap[pos.x][pos.y-1] + singleLaneMap[pos.x][pos.y+1];
        if(left == -1 && up == -1){
            //拐角
            return true;
        }
        // LOGI("上一个id：",nextSingleLaneId,",当前坐标：",pos);
        // LOGI("left:",left,",up:",up);
        LOGI(singleLaneMap[pos.x-1][pos.y],",",singleLaneMap[pos.x+1][pos.y],",",singleLaneMap[pos.x][pos.y-1],",",singleLaneMap[pos.x][pos.y+1]);
        return false;
    }

    // 空地或者泊位用0初始化，其他用-1初始化
    void initMap(){
        for(int i=0;i < rows; i++){
            for(int j=0;j < cols;j++){
                if(grid[i][j] == MapItemSpace::MapItem::SPACE ||grid[i][j] == MapItemSpace::MapItem::BERTH){
                    singleLaneMap[i][j] = 0;
                }
                else{
                    singleLaneMap[i][j] = -1;
                }
            }
        }
    }

    // 是否属于单行路
    // 返回当前格子障碍数
    int countObstacle(Point2d & point){
        std::vector<Point2d> temp;
        temp.push_back(Point2d(point.x - 1,point.y));
        temp.push_back(Point2d(point.x + 1,point.y));
        temp.push_back(Point2d(point.x,point.y - 1));
        temp.push_back(Point2d(point.x,point.y + 1));
        int num = 0;
        for(auto& item : temp){
            if(!isValid(item) || !canPass(item)){
                num++;
            }
        }
        // LOGI(point,",当前位置周围障碍数量:",num);
        return num;
    }


    // 只有两个障碍就是可以通行
    void findSingleLaneFromPoint(Point2d pos, std::vector<Point2d>& path) {
        // 路到尽头了
        //该格子周围障碍数小于两个才是单行路
        isValid(pos);
        // LOGI("有效");
        visited[pos.x][pos.y] == VisitType::VISITED;
        // LOGI("访问map有用");
        if (!isValid(pos) || visited[pos.x][pos.y] == VisitType::VISITED) return;
        visited[pos.x][pos.y] = VisitType::VISITED;
        // LOGI("有效");
        if (!canPass(pos)){
            return;
        }
        // LOGI("可通行");
        int num = countObstacle(pos);
        if(num < 2 || num ==4) return;
        // LOGI(pos,",当前位置周围障碍数量:",countObstacle(pos));
        
        path.push_back(pos);

        std::vector<Point2d> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        // int validDirections = 0;
        Point2d nextStep;

        for (const auto& dir : directions) {
            Point2d nextPos = {pos.x + dir.x, pos.y + dir.y};
            if (isValid(nextPos) && canPass(nextPos) && visited[nextPos.x][nextPos.y] == VisitType::UNVISITED) {
                // validDirections++;
                // if(nextPos.x >=199){LOGI("debug：",nextPos,",周围障碍：",countObstacle(nextPos));}
                nextStep = nextPos;
                findSingleLaneFromPoint(nextStep, path);
            }
        }
    }

    // 找到所有单行路
    void findAllSingleLanes() {
        for (int x = 0; x < rows; ++x) {
            for (int y = 0; y < cols; ++y) {
                auto start = std::chrono::steady_clock::now();
                if (canPass({x, y}) && visited[x][y] == VisitType::UNVISITED) {
                    std::vector<Point2d> path;
                    findSingleLaneFromPoint({x, y}, path);
                    if (path.size() >= 2 || (path.size() == 1 && ! isCorner(path[0]))) {
                        int laneId = nextSingleLaneId++;
                        singleLanes[laneId] = path; // 保存找到的单行路路径
                        singleLaneLocks[laneId] = SingleLaneLock(path[0],path[path.size()-1]); // 默认锁为解锁状态
                        // 对单行路地图进行标记
                        for(auto& point : path){
                            singleLaneMap[point.x][point.y] = laneId;
                        }
                    }
                }
                auto end = std::chrono::steady_clock::now();
                int findTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                // LOGI("当前寻找坐标：",x,",",y,",耗费时间：",findTime);
            }
        }
        LOGI("寻找单行路循环完毕！");
    }

};