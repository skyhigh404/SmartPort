#pragma once
#include <string>

#include "goods.h"
#include "map.h"
#include "utils.h"
#include "pathFinder.h"
#include "assert.h"
#include "log.h"

enum RobotStatus
{
    IDLE,
    MOVING_TO_GOODS,
    MOVING_TO_BERTH,
    UNLOADING,
    DEATH,
    DIZZY
};

class Robot
{
public:
    // 判题器输入数据
    int id;
    Point2d pos;
    int carryingItem; // 0 表示未携带物品，1 表示携带物品
    int state;        // 0 表示恢复状态，1 表示正常运行状态
public:
    RobotStatus status;
    int carryingItemId; // 携带的物品id
    int targetid;
    Point2d destination; // 机器人当前的目的地
    Point2d nextPos;
    std::vector<Point2d> path; // 机器人即将要走的路径
private:
    // DStarPathfinder pathFinder; // 每个机器人都要存储寻路状态
    AStarPathfinder pathFinder;

public:
    Robot(int id, Point2d pos)
        : id(id),
          pos(pos),
          carryingItem(0),
          state(0),
          status(IDLE),
          carryingItemId(-1),
          targetid(0),
          nextPos(-1,-1)
    {
    }

    void updatePath()
    {
        // 正常移动
        if(nextPos == pos && !path.empty() && nextPos==path.back()){
            path.pop_back();
        }
        // 没有移动到预定位置
        else if(nextPos != Point2d(-1,-1) &&nextPos != pos){
            LOGW(id, " 没有移动到预定位置, current pos: ", pos, " next pos: ", nextPos);
        }
    }

    void updateNextPos()
    {
        if (!path.empty() && status != DIZZY)
            // 寻路算法输出的路径是逆序存储的，以提高弹出效率
            nextPos = this->path.back();
        // 如果路径为空，则机器人下一帧不移动
        else
            nextPos = pos;
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
        if(nextPos != Point2d(-1, -1) && Point2d::calculateManhattanDistance(nextPos, pos) == 1)
            return move(nextPos);
        LOGW("robot ",id, " from: ",pos , " to ", nextPos);
        return "";
    }

    std::string moveWithPath()
    {
        if (!path.empty())
        {
            // A* 算法输出的路径是逆序存储的，以提高弹出效率
            nextPos = this->path.back();
            return move(nextPos);
        }
        return std::string("");
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
        std::variant<Path, PathfindingFailureReason> path = pathFinder.findPath(pos, destination, map);
        if (std::holds_alternative<Path>(path)){
            this->path = std::get<Path>(path);
            return true;
        }
        else {
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
    // 认定正常运行的，路径更短的优先级更高，然后是不携带货物的优先级更高，最后比较 ID，更小的优先级更高
    bool comparePriority(const Robot &rhs) const
    {
        if(state != rhs.state)
            return state > rhs.state;
        else if (path.size() != rhs.path.size())
            return path.size() < rhs.path.size();
        else if (carryingItem != rhs.carryingItem)
            return carryingItem < rhs.carryingItem;
        else
            return id < rhs.id;
    }
    
    friend std::ostream &operator<<(std::ostream &os, const Robot &robot) {
        os << "id: " << robot.id << " pos: " << robot.pos << " nextPos: " << robot.nextPos << " path: ";
        for(auto & item : robot.path){
            os << item;
        }
        return os;
    }
};
