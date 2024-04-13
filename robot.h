#pragma once
#include <string>

#include "goods.h"
#include "map.h"
#include "utils.h"
#include "pathFinder.h"
#include "assert.h"
#include "log.h"

// 指示机器人当前的状态
enum RobotStatus
{
    IDLE,            // 机器人处于空闲状态，等待新的任务分配
    MOVING_TO_GOODS, // 机器人正在移动至指定货物位置。
    MOVING_TO_BERTH, // 机器人正在移动至指定泊位。
    UNLOADING,       // 机器人正在卸载货物。
    DEATH,           // 机器人无法工作。
};

class Robot
{
public:
    // 判题器输入数据
    int id;                 // 机器人 ID
    Point2d pos;            // 机器人目前位置
    int type = 0;               // 0\1
    int carryingItem;       // 0 表示未携带物品，1 表示携带1个物品，2表示携带2个物品
    // const int price = 2000; // 购买价格
    // int state;        // 0 表示恢复状态，1 表示正常运行状态
public:
    RobotStatus status;
    int carryingItemId;  // 携带的物品 ID
    int carryingItemId2 = -1; // 携带的第二个物品ID
    int targetid;        // 机器人目标货物或泊位的 ID
    Point2d destination; // 机器人的目的地
    // BerthID assignedBerthID; // 机器人被分配的泊位 ID
public:
    Point2d nextPos;           // 机器人下一帧前往的位置
    std::vector<Point2d> path; // 机器人运行路径
    int avoidNum = 0;          //  避让的次数
private:
    // DStarPathfinder pathFinder; // 每个机器人都要存储寻路状态
    AStarPathfinder<Point2d, Map> pathFinder;

public:
    Robot(int id, Point2d pos)
        : id(id),
          pos(pos),
          carryingItem(0),
          status(RobotStatus::IDLE),
          carryingItemId(-1),
          targetid(-1),
          nextPos(-1, -1)
    {
    }

    //  购买机器人
    static std::string lbot(const Point2d &pos, const int type)
    {
        using namespace std::string_literals;
        // LOGI("lbot "s + std::to_string(pos.x) + " "s + std::to_string(pos.y) + " "s + std::to_string(type));
        return "lbot "s + std::to_string(pos.x) + " "s + std::to_string(pos.y) + " "s + std::to_string(type);
    }

    void assignGoodOrBerth()
    {
        targetid = -1;
        destination = Point2d(-1, -1);
    }
    void assignGoodOrBerth(int Id, Point2d dest)
    {
        targetid = Id;
        destination = dest;
    }

    void updatePath()
    {
        // 正常移动
        if (nextPos == pos && !path.empty() && nextPos == path.back())
        {
            path.pop_back();
        }
        // 没有移动到预定位置
        else if (nextPos != Point2d(-1, -1) && nextPos != pos)
        {
            LOGW("Robot ", id, " 没有移动到预定位置, current pos: ", pos, " next pos: ", nextPos);
        }
    }

    void updateNextPos()
    {
        if (!path.empty())
            // 寻路算法输出的路径是逆序存储的，以提高弹出效率
            nextPos = this->path.back();
        // 如果路径为空，则机器人下一帧不移动
        else
            nextPos = pos;
    }

    void moveToTemporaryPosition(const Point2d &tempPos)
    {
        path.push_back(pos);
        // 让机器人让路后多停一帧
        path.push_back(tempPos);
        path.push_back(tempPos);
        nextPos = tempPos;
    }

    std::string move(int direction)
    {
        // 向特定方向移动
        using namespace std::string_literals;
#ifdef DEBUG
        assert(direction >= 0 && direction <= 3);
#endif
        return "move "s + std::to_string(id) + " "s + std::to_string(direction);
    }

    std::string move(const Point2d &nextPos)
    {
        std::string instruction;
        if (nextPos.x > pos.x)
            instruction = move(3); // 向下
        else if (nextPos.x < pos.x)
            instruction = move(2); // 向上
        else if (nextPos.y > pos.y)
            instruction = move(0); // 向右
        else if (nextPos.y < pos.y)
            instruction = move(1); // 向左
        return instruction;
    }

    std::string movetoNextPosition()
    {
        if (nextPos != Point2d(-1, -1) && Point2d::calculateManhattanDistance(nextPos, pos) <= 1)
            return move(nextPos);
        LOGW("robot ", id, " from: ", pos, " to ", nextPos);
        return "";
    }

    std::string get()
    {
        // 生成取货指令
        using namespace std::string_literals;
        return "get "s + std::to_string(id);
    }

    std::string pull()
    {
        // 生成放置指令
        using namespace std::string_literals;
        return "pull "s + std::to_string(id);
    }

    bool findPath(const Map &map, Point2d dst)
    {
        destination = dst;
        std::variant<Path<Point2d>, PathfindingFailureReason> path = pathFinder.findPath(pos, destination, map);
        if (std::holds_alternative<Path<Point2d>>(path))
        {
            this->path = std::get<Path<Point2d>>(path);
            return true;
        }
        else
        {
            return false;
        }
        // if(pathFinder.sameGoal(destination)){
        //     // std::vector<Point2d> potentialObstacle = map.isCollisionRisk(id, 1);
        //     // // 如果有碰撞风险
        //     // if(potentialObstacle.size() > 0){
        //     //     // 如果目标被占据
        //     //     if(Point2d::isIN(destination, potentialObstacle)){
        //     //         // 原地等待
        //     //         LOGI("Robot ", id, " 原地等待"," pos: ", pos, "dst: ", destination, " path: ", printVector(path));
        //     //         path.push_back(pos);
        //     //         return true;
        //     //     }
        //     //     LOGI("Robot ", id, " path: ", printVector(path));
        //     //     LOGI("Robot ", id, " pos: ", pos, "dst: ", destination, " 开始避让，潜在障碍位置: ", printVector(potentialObstacle));
        //     //     path = pathFinder.replan(pos, map, potentialObstacle, std::vector<bool>(potentialObstacle.size(), true));
        //     //     LOGI("Robot ", id, " replan path: ", printVector(path));

        //     refindPath(map);
        //     // }
        // }
        // else{
        //     // 给定了新的终点
        //     path = pathFinder.plan(this->pos, this->destination, map);
        // }
        // if (!path.empty())
        //     return true;
        // else
        //     return false;
    }

    bool findPath(const Map &map)
    {
        return findPath(map, destination);
    }

    bool refindPath(const Map &map)
    {
        std::vector<Point2d> potentialObstacle = map.getNearbyTemporaryObstacles(pos, 2);
        // path = pathFinder.replan(pos, map, potentialObstacle, std::vector<bool>(potentialObstacle.size(), true));
        if (!path.empty())
            return true;
        else
            return false;
    }

    // 判断优先级，如果优先级更大则返回 true
    // 位于单行路内或者路口的优先级最高
    // 认定正常运行的，不携带货物的优先级更高，然后是路径更短的优先级更高，最后比较 ID，更小的优先级更高
    bool comparePriority(const Robot &rhs) const
    {
        if (carryingItem != rhs.carryingItem)
            return carryingItem < rhs.carryingItem;
        else if (path.size() != rhs.path.size())
            return path.size() < rhs.path.size();
        else
            return id < rhs.id;
    }

    // 获取未来 n 帧 机器人路径上的点
    std::vector<Point2d> getLastPathPoint(size_t n) const
    {
        if (n <= path.size())
            return std::vector<Point2d>(path.end() - n, path.end());
        else
            return path;
    }

    friend std::ostream &operator<<(std::ostream &os, const Robot &robot)
    {
        os << "id: " << robot.id << " pos: " << robot.pos << " nextPos: " << robot.nextPos << " dst: " << robot.destination << " path: " << robot.path.size() << ", ";
        for (int i = (int)robot.path.size() - 1; i >= std::max(0, (int)robot.path.size() - 3); --i)
        {
            os << robot.path[i];
        }
        return os;
    }
};
