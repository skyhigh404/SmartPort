#pragma once
#include <set>
#include "ship.h"
#include "utils.h"
#include "seaSingleLaneManager.h"
#include "log.h"

class ShipController
{
public:
    struct CollisionEvent 
    {
        int shipId1; // 第一个船的ID
        int shipId2; // 第二个船的ID，如果事件只涉及一个船，则此ID可以设置为-1
        enum CollisionType { 
            // 优先级从高到低
            EntryAttemptWhileOccupied, // 单行道已被占据，另一船尝试进入导致冲突
            NextOverlapCollision,  // 两艘船下一帧位置冲突
            PathCrossingCollision,  // 两艘船下一帧位置不冲突，但是由于执行顺序产生的冲突
        } type; // 碰撞类型
        
        CollisionEvent(int id1, CollisionType t)
            : shipId1(id1), shipId2(-1), type(t) {}
        CollisionEvent(int id1, int id2, CollisionType t)
            : shipId1(std::min(id1, id2)), shipId2(std::max(id1, id2)), type(t) {}
        bool operator<(const CollisionEvent &rhs) const {
            // 防止重复船对
            return shipId1 < rhs.shipId1 || (shipId1 == rhs.shipId1 && shipId2 < rhs.shipId2);
        }
    };

    struct CollisionEventCompare {
        // set 中使用
        bool operator()(const CollisionEvent& lhs, const CollisionEvent& rhs) const {
        return lhs.type < rhs.type;
        }
    };

    // 解决冲突方法
    struct ResolutionAction
    {
        enum Method  {
            // 优先级从高到低
            Continue,   // 继续移动，目前未使用
            Dept,    // 死锁时重置到主航道
            Wait,       // 等待
            // MoveAside,  // 往空位移动一格以让路，需要新的解决方法
            RefindPath,  // 重新寻路,
            
        } method;

        // Point2d moveTo; // 只有在MoveAside时使用
        ResolutionAction(Method m) : method(m) {}
    };
    
    

public:
    // ShipController(std::vector<Ship> &ships) : ships(ships) {}
    ShipController() {}

    // 为所有需要寻路算法的船调用寻路算法
    // 更新所有船下一步位置
    // while
    // 考虑下一步船的行动是否会冲突
    // 解决冲突（重新寻路或等待，根据它们的代价来判断）
    // 直至解决冲突
    // 确定下一步所有船的行动
    void runController(Map &map,std::vector<Ship> &ships, SeaSingleLaneManager &seaSingleLaneManager);

private:
    void reset(){
        shipResolutionActions.clear();
    }
    // 更新地图上的临时障碍物
    void updateTemporaryObstacles(Map &map, std::vector<Ship> &ships);
    // 根据动作判断是否需要调用寻路算法
    bool needPathfinding(Ship &ship);


    // 检测船之间是否冲突，输出冲突的船 ID (对)，不考虑地图障碍物的情况
    std::set<CollisionEvent, CollisionEventCompare> detectNextFrameConflict(Map &map, std::vector<Ship> &ships, SeaSingleLaneManager &seaSingleLaneManager);

    // 尝试为所有船分配新状态解决冲突
    void tryResolveConflict(Map &map, std::vector<Ship> &ships, const CollisionEvent &event);
    // 根据船的 ShipResolutionActions 权衡合理的规划逻辑
    void rePlanShipMove(Map &map, std::vector<Ship> &ships);

    // 解决SwapPositions死锁的逻辑，尝试让一个船移动往一个可行的点以让出终点
    void resolveDeadlocks(Map &map, Ship &ship1, Ship &ship2);
    // 处理两个船下一帧目标重合的冲突函数
    void decideWhoToWaitAndRefindWhenTargetOverlap(Map &map, Ship &ship1, Ship &ship2);
    // 根据船优先级判断应该等待的船的引用，返回优先级低的船
    const Ship & decideWhoWaits(Ship &ship1, Ship &ship2);

    // 设置标志位，让一个船等待
    void makeShipWait(const Ship &ship);
    // 设置标志位，让一个船重新寻路
    void makeShipRefindPath(const Ship &ship);
    // 设置标志位，让一个船移动到临时位置
    void makeShipMoveToTempPos(const Ship &ship);
    // 设置标志位，让船瞬移到最近的主航道
    void makeShipDept(const Ship &ship);

    // 让一个船等待
    void stopShip(Ship &ship);
    // 让一个船离港
    void deptShip(Ship &ship);
    // 让一个船移动往除了下一帧外的另一个位置
    Point2d moveAsideShip(const Map &map, Ship &ship);
    // 让一个船寻路
    void runPathfinding(const Map &map, Ship &ship);


    // 判断点是否在运行轨迹内
    bool pointInTrajectory(const Point2d &pos, const std::vector<Point2d> traj);

    // 判断两个核心点的方向是否平行
    bool isParallel(Direction &a, Direction &b){
        if (abs(static_cast<int>(a) - static_cast<int>(b)) >= 2) return false;
        else return true;
    }

    // 判断两个核心点的方向是否垂直
    bool isVertical(Direction &a, Direction &b){
        if (abs(static_cast<int>(a) - static_cast<int>(b)) >= 2) return true;
        else return false;
    }

    // // 判断船的位置是否发生冲突
    // bool hasOverlap(Map &map, VectorPosition &a, VectorPosition &b);

    // 判断两艘船下一帧位置是否冲突
    bool isNextOverlapCollision(Map &map, Ship &a, Ship &b);

    // 判断两艘船是否是由于执行顺序而引起的重推
    bool isPathCrossingCollision(Map &map, Ship &a, Ship &b);

    // 判断船是否前往一个泊位

    
private:
    // std::vector<Ship> &ships;
    std::unordered_map<int, std::vector<ResolutionAction>> shipResolutionActions;
};