#pragma once

#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include "utils.h"
#include "map.h"
#include "robot.h"

struct SeaSingleLaneLock
{
    std::pair<Point2d, Point2d> startPos;
    std::pair<Point2d, Point2d> endPos;
    // Point2d entrance;   //startPos往外扩展一格
    // Point2d exit;   //endPos的往外扩展一格

    bool startLock;
    bool endLock;
    int count;
    SeaSingleLaneLock(std::pair<Point2d, Point2d> start,std::pair<Point2d, Point2d> end):startPos(start),endPos(end),startLock(false),endLock(false),count(0){}
    SeaSingleLaneLock(){}

    inline bool isDeadEnd() const {
        return endPos.first == Point2d(-1, -1) || endPos.second == Point2d(-1, -1);
    }

    // 加锁函数，参数指明是从startPos进入还是从endPos进入
    void lock(bool fromStart) {
        if (isDeadEnd()) {
            // 对于死路，无论从哪个方向进入，都只锁定startLock
            startLock = true;
        }
        // 给进入的另一端加锁
        else {
            if (fromStart)
                endLock = true;
            else
                startLock = true;
        }
        count++; // 增加在单行道内的机器人数量
    }

    // 释放锁函数，参数指明是从startPos退出还是从endPos退出
    void unlock(bool fromStart) {
        count--; // 减少在单行道内的机器人数量
        if (count == 0) { // 如果单行道为空，则解锁两端
            startLock = false;
            endLock = false;
        }
        else if (count < 0) {
            startLock = false;
            endLock = false;
            // 机器人可能就出生在单行道内
            LOGE("错误的释放单行路锁, count: ", count);
            count = 0;
        }
    }
};



class SeaSingleLaneManager {
public:
    int rows, cols;
    std::vector<std::vector<int>> singleLaneMap;    //  标记为单行路的id(从1开始)，标记0为正常水路，标记-1为障碍
    std::unordered_map<int,SeaSingleLaneLock> singleLaneLocks;    // 维护每个单行路的锁，标记当前是否通行
    std::unordered_map<int,std::vector<VectorPosition>> singleLanes;   //存储单行路的路径

    std::vector<std::vector<MapItemSpace::MapItem>> grid;    //原地图
    std::unordered_map<Direction,std::vector<std::vector<VisitType>>> visited; // 访问过的位置(分为x,y轴两个方向)

    

    int nextSingleLaneId = 1;   // 单行路id，从1开始

    SeaSingleLaneManager(){}

    SeaSingleLaneManager(const Map& map){
        this->rows = map.rows;
        this->cols = map.cols;
        this->grid = map.grid;
        singleLaneMap = std::vector<std::vector<int>>(map.rows,std::vector<int>(map.cols,-1));
        visited[Direction::EAST] = visited[Direction::WEST] = std::vector<std::vector<VisitType>>(map.rows,std::vector<VisitType>(map.cols,VisitType::UNVISITED));
        visited[Direction::NORTH] = visited[Direction::SOUTH] = std::vector<std::vector<VisitType>>(map.rows,std::vector<VisitType>(map.cols,VisitType::UNVISITED));

        // 找到所有单行路
        findAllSingleLanes(map);
    }

    // void lock(int laneId, const Point2d& entryPoint) {
    //     if (singleLaneLocks.find(laneId) != singleLaneLocks.end()) {
    //         SeaSingleLaneLock& lock = singleLaneLocks.at(laneId);
    //         // 不对死路进行特殊处理
    //         if (entryPoint == lock.startPos)
    //             lock.lock(true);
    //         else if (entryPoint == lock.endPos)
    //             lock.lock(false);
    //         else
    //             LOGW("传入错误的加锁位置, pos: ", entryPoint);
    //     }
    //     else {
    //         LOGE("尝试从singleLaneLocks获取不存在的锁的情况 laneId: ", laneId);
    //     }
    // }

    // void unlock(int laneId, const Point2d& entryPoint) {
    //     if (singleLaneLocks.find(laneId) != singleLaneLocks.end()) {
    //         SeaSingleLaneLock& lock = singleLaneLocks.at(laneId);
    //         // 不对死路进行特殊处理
    //         if (entryPoint == lock.startPos)
    //             lock.unlock(true);
    //         else if (entryPoint == lock.endPos)
    //             lock.unlock(false);
    //         else
    //             LOGW("传入错误的解锁位置, pos: ", entryPoint);
    //     }
    //     else {
    //         LOGE("尝试从singleLaneLocks获取不存在的锁的情况 laneId: ", laneId);
    //     }
    // }

    inline int getSingleLaneId(const Point2d& point) const {
        if (point.x >= 0 && point.x < rows && point.y >= 0 && point.y < cols) {
            return singleLaneMap[point.x][point.y];
        }
        return -1; // 超出边界
    }

    // // 根据提供的位置，判断是否在单行道的入口处
    // bool isEnteringSingleLane(int laneId, const Point2d& entryPoint) const {
    //     if (singleLaneLocks.find(laneId) != singleLaneLocks.end()) {
    //         const SeaSingleLaneLock& lock = singleLaneLocks.at(laneId);
    //         // 不对死路进行特殊处理
    //         if(entryPoint == lock.startPos || entryPoint == lock.endPos)
    //             return true;
    //     }
    //     else {
    //         LOGE("尝试从singleLaneLocks获取不存在的锁的情况 laneId: ", laneId);
    //     }
    //     return false;
    // }
    // // 提供单行路的进入点，检查单行路是否被锁定
    // bool isLocked(int laneId, const Point2d& entryPoint) const {
    //     if (singleLaneLocks.find(laneId) != singleLaneLocks.end()) {
    //         const SeaSingleLaneLock& lock = singleLaneLocks.at(laneId);
    //         if (lock.isDeadEnd()) {
    //             // 对于死路，只检查startLock
    //             return lock.startLock;
    //         } 
    //         else {
    //             // 检查进入方向是否被加锁
    //             if (lock.startPos == entryPoint && lock.startLock)
    //                 return true;
    //             if (lock.endPos == entryPoint && lock.endLock)
    //                 return true;
    //         }
    //     }
    //     else {
    //         LOGE("尝试从singleLaneLocks获取不存在的锁的情况 laneId: ", laneId);
    //     }
    //     return false;
    // }

    SeaSingleLaneLock& getLock(int laneId) {
        return singleLaneLocks[laneId];
    }

    void init(const Map& map){
        this->rows = map.rows;
        this->cols = map.cols;
        // 拷贝复制
        this->grid = std::vector<std::vector<MapItemSpace::MapItem>>(map.grid);
        singleLaneMap = std::vector<std::vector<int>>(map.rows,std::vector<int>(map.cols,-1));
        visited[Direction::EAST] = visited[Direction::WEST] = std::vector<std::vector<VisitType>>(map.rows,std::vector<VisitType>(map.cols,VisitType::UNVISITED));
        visited[Direction::NORTH] = visited[Direction::SOUTH] = std::vector<std::vector<VisitType>>(map.rows,std::vector<VisitType>(map.cols,VisitType::UNVISITED));
        // 初始化地图
        initMap();
        // 找到所有单行路
        findAllSingleLanes(map);
    }

    bool isValid(const Point2d& pos) { return pos.x >= 0 && pos.x < rows && pos.y >= 0 && pos.y < cols; }

    // 传入核心点坐标，获取船的占用空间，判断是否全部可用
    bool canSeaPass(const Map &map, const VectorPosition& vecPos) {
        // return grid[pos.x][pos.y] == MapItemSpace::MapItem::SPACE || grid[pos.x][pos.y] == MapItemSpace::MapItem::BERTH;
        std::pair<Point2d,Point2d> shipSpace = SpatialUtils::getShipOccupancyRect(vecPos);
        for(int x = shipSpace.first.x; x < shipSpace.second.x; x++){
            for( int y = shipSpace.second.y; y < shipSpace.second.y; y++){
                // 节点无效 || 不可通行 || 在主航道内
                // todo 后期考虑是否去掉 主航道内判断
                if (!map.inBounds(Point2d(x, y)) || !map.seaPassable(Point2d(x, y)) || map.isInSealane(Point2d(x,y)))
                    return false;
            }
        }
        return true;
    }

    // 传入船的占地空间，判断是否全部可用
    bool canSeaPass(const Map &map, const std::pair<Point2d,Point2d> shipSpace) {
        // return grid[pos.x][pos.y] == MapItemSpace::MapItem::SPACE || grid[pos.x][pos.y] == MapItemSpace::MapItem::BERTH;
        for(int x = shipSpace.first.x; x < shipSpace.second.x; x++){
            for( int y = shipSpace.second.y; y < shipSpace.second.y; y++){
                // 节点有效并可同行
                if (!map.inBounds(Point2d(x, y)) || !map.seaPassable(Point2d(x, y)))
                    return false;
            }
        }
        return true;
    }

    // 判断是否是拐角
    bool isCorner(Point2d &pos){
        int left = 0,up = 0;
        if(!isValid({pos.x-1,pos.y})) left += -1;
        else left += singleLaneMap[pos.x-1][pos.y];
        if(!isValid({pos.x+1,pos.y})) left += -1;
        else left += singleLaneMap[pos.x+1][pos.y];

        if(!isValid({pos.x,pos.y-1})) up += -1;
        else up += singleLaneMap[pos.x][pos.y-1];
        if(!isValid({pos.x,pos.y+1})) up += -1;
        else up +=singleLaneMap[pos.x][pos.y+1];
        if(left == -1 && up == -1){
            //拐角
            return true;
        }
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
    // 传入核心点，判断该路是否仅容纳一艘船通过
    int canAccommodateSingleShip(const Map &map, VectorPosition &vecPos){
        // visited[vecPos.direction][vecPos.pos.x][vecPos.pos.y] = VisitType::VISITED;
        // 判断当前位置是否可通行
        if (!canSeaPass(map, vecPos)) return false;
        
        // 判断周围是否有通行空间
        std::vector<VectorPosition> corePointList;
        std::unordered_map<Direction,std::vector<Point2d>> coreOffsetByDirection;
        coreOffsetByDirection[Direction::EAST] = coreOffsetByDirection[Direction::WEST] = std::vector<Point2d>{{0,1},{0,2},{0,-1},{0,-2}};
        coreOffsetByDirection[Direction::NORTH] = coreOffsetByDirection[Direction::SOUTH] = std::vector<Point2d>{{1,0},{2,0},{-1,0},{-2,0}};
        for(auto &offset : coreOffsetByDirection[vecPos.direction]){
            corePointList.push_back(VectorPosition(vecPos.pos + offset, vecPos.direction));
        }
        int num = 0;
        // 超过两个可以通行，则失败
        for(auto &corePoint : corePointList){
            if(map.inBounds(corePoint.pos) && canSeaPass(map, vecPos)) num++;
            // todo 缩小搜索空间
            visited[vecPos.direction][corePoint.pos.x][corePoint.pos.y] = VisitType::VISITED;
        }
        if(num >= 2) return false;
        else return true;
    }

    // 只有两个障碍就是可以通行
    void findSingleLaneFromPoint(const Map &map, VectorPosition vecPos, std::vector<VectorPosition>& path,bool flag) {
        // 路到尽头了
        if (!map.inBounds(vecPos.pos) || visited[vecPos.direction][vecPos.pos.x][vecPos.pos.y] == VisitType::VISITED) return;
        visited[vecPos.direction][vecPos.pos.x][vecPos.pos.y] = VisitType::VISITED;
        // 在主航道上
        if (map.isInSealane(vecPos.pos)) return;
        
        // 不满足：只容纳一艘船通过
        if (!canAccommodateSingleShip(map, vecPos)){
            return;
        }
        if (flag){
            path.push_back(vecPos);
        }
        else{
            path.insert(path.begin(),vecPos);
        }
        

        // std::vector<Point2d> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        // int validDirections = 0;
        // Point2d nextStep;
        std::vector<VectorPosition> nextSteps = std::vector<VectorPosition>{SpatialUtils::moveForward(vecPos),
        SpatialUtils::clockwiseRotation(vecPos), SpatialUtils::anticlockwiseRotation(vecPos)};  // 前进一格、顺时针、逆时针，不考虑后退

        for (const auto& nextPos : nextSteps) {
            if (map.inBounds(nextPos.pos) && canSeaPass(map, nextPos) && visited[nextPos.direction][nextPos.pos.x][nextPos.pos.y] == VisitType::UNVISITED) {
                int temp_size = path.size();
                findSingleLaneFromPoint(map, nextPos, path,flag);
                if(temp_size != path.size()) flag = !flag;
            }
        }
    }

    // 传入核心点坐标，返回和前进方向垂直的边界坐标
    std::pair<Point2d, Point2d> getBorderPos(const Map &map, VectorPosition vecPos){
        std::pair<Point2d, Point2d> borderPos = {Point2d(vecPos.pos), Point2d(vecPos.pos)};
        int count = 0;  //限制寻找的次数
        if(vecPos.direction == Direction::EAST || vecPos.direction == Direction::WEST){
            
            while(map.inBounds(borderPos.first) && map.seaPassable(borderPos.first) && !map.isInSealane(borderPos.first)){
                borderPos.first.y -= 1;

                count++;
                if(count > 5) {count = 0;break;}
            }
            borderPos.first.y += 1;
            while(map.inBounds(borderPos.second) && map.seaPassable(borderPos.second) && !map.isInSealane(borderPos.second)){
                borderPos.second.y += 1;

                count++;
                if(count > 5) {count = 0;break;}
            }
            borderPos.second.y -= 1;
        }else{
            while(map.inBounds(borderPos.first) && map.seaPassable(borderPos.first) && !map.isInSealane(borderPos.first)){
                borderPos.first.x -= 1;

                count++;
                if(count > 5) {count = 0;break;}
            }
            borderPos.first.x += 1;
            while(map.inBounds(borderPos.second) && map.seaPassable(borderPos.second) && !map.isInSealane(borderPos.second)){
                borderPos.second.x += 1;

                count++;
                if(count > 5) {count = 0;break;}
            }
            borderPos.second.x -= 1;
        }
        return borderPos;
    }


    // 找到所有单行路
    void findAllSingleLanes(const Map &map) {
        for (int x = 0; x < rows; ++x) {
            for (int y = 0; y < cols; ++y) {
                auto start = std::chrono::steady_clock::now();
                if (canSeaPass(map, VectorPosition({x, y},Direction::EAST)) && visited[Direction::EAST][x][y] == VisitType::UNVISITED) {
                    std::vector<VectorPosition> path;
                    findSingleLaneFromPoint(map, VectorPosition({x, y}, Direction::EAST), path,true);
                    if (path.size() != 0) {
                        int laneId = nextSingleLaneId++;
                        singleLanes[laneId] = path; // 保存找到的单行路路径
                        // singleLaneLocks[laneId] = SeaSingleLaneLock(path[0],path.back()); // 默认锁为解锁状态
                        // 对单行路地图进行标记
                        for(auto& point : path){
                            // 获取边界坐标
                            std::pair<Point2d, Point2d> borderPos = getBorderPos(map, point);
                            if (point.direction == Direction::EAST || point.direction == Direction::WEST){
                                for (int i = borderPos.first.y; i <= borderPos.first.y; i++) singleLaneMap[point.pos.x][i] = laneId;
                            }
                            else{
                                for (int i = borderPos.first.x; i <= borderPos.first.x; i++) singleLaneMap[i][point.pos.y] = laneId;
                            }
                            // LOGI(point);
                        }

                        // if(countObstacle(path[0]) == 3){
                        //     singleLaneLocks[laneId].startPos = singleLaneLocks[laneId].endPos;
                        //     singleLaneLocks[laneId].endPos = {-1,-1};
                        //     std::reverse(std::begin(singleLanes[laneId]),std::end(singleLanes[laneId]));
                        // }
                        // if(countObstacle(path.back()) == 3){
                        //     singleLaneLocks[laneId].endPos = {-1,-1};
                        // }
                    }
                }
                auto end = std::chrono::steady_clock::now();
                int findTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                // LOGI("当前寻找坐标：",x,",",y,",耗费时间：",findTime);
            }
        }
        LOGI("寻找水路单行路循环完毕！");
    }   

};