#pragma once

#include <string>
#include "utils.h"
#include "log.h"
#include "map.h"
#include "pathFinder.h"
#include "assert.h"

class Ship
{
public:
    int id;
    int goodsCount;           // 携带的货物数量
    VectorPosition locAndDir; //  表示船舶位置和方向
    int state;                // 0: 正常行驶状态, 1: 恢复状态, 2: 装载状态
    int berthId;              // 目标泊位 ID
    // const int price = 8000;       // 购买价格
public:
    static int capacity;        // 船的容量
    int remainingTransportTime; // 船到目标泊位的剩余运行时间，在处理每一帧信息时维护
    VectorPosition destination;

public:
    VectorPosition nextLocAndDir;     // 船舶下一帧位姿
    std::vector<VectorPosition> path; // 船舶运行路径
    int avoidNum = 0;                 //  避让的次数
private:
    AStarPathfinder<VectorPosition, Map> pathFinder;

public:
    Ship(int id)
        : id(id),
          state(0),
          berthId(-1),
          remainingTransportTime(0),
          nextLocAndDir(-1, -1, Direction::EAST) {}

    //  购买船只
    static std::string lboat(const Point2d &pos)
    {
        using namespace std::string_literals;
        return "lboat "s + std::to_string(pos.x) + " "s + std::to_string(pos.y);
    }

    // 重置船只到主航道
    std::string dept()
    {
        using namespace std::string_literals;
#ifdef DEBUG
        // 船不在恢复状态
        assert(state != 1);
#endif
        return "dept "s + std::to_string(id);
    }

    // 尝试将对应船靠泊到泊位上，会导致船进入恢复状态。
    std::string berth()
    {
        using namespace std::string_literals;
#ifdef DEBUG
        // 船不在恢复状态
        assert(state != 1);
#endif
        return "berth "s + std::to_string(id);
    }

    // 旋转命令
    std::string rot(RotationDirection rotDirection)
    {
        using namespace std::string_literals;
#ifdef DEBUG
        // 船在正常行驶状态
        assert(state == 0);
#endif
        return "rot "s + std::to_string(static_cast<int>(rotDirection));
    }

    // 前进命令
    std::string ship()
    {
        using namespace std::string_literals;
#ifdef DEBUG
        // 船在正常行驶状态
        assert(state == 0);
#endif
        return "ship "s + std::to_string(id);
    }

    // 装货,并返回转货的数量
    int loadGoods(int num)
    {
#ifdef DEBUG
        assert(nowCapacity() >= 0);
#endif
        if (nowCapacity() == 0)
        {
            // 异常情况，满货船舶停滞在泊位
            return 0;
        }
        else if (nowCapacity() >= num)
        {
            return num;
        }
        else
        {
            return nowCapacity();
        }
    }

    // 打印信息
    void info()
    {
        LOGI("船只", id, ",状态", state, ",装货量：", capacity, ",剩余容量：", nowCapacity(), ",剩余容量比例：", nowCapacity() * 1.0 / capacity, ",泊位id：", berthId, ";");
    }

    float capacityScale()
    {
        return 1.0 * nowCapacity() / capacity;
    }

    // 获取船的剩余容量
    inline int nowCapacity()
    {
        return std::max(capacity - goodsCount, -1);
    }

    // 寻路
    bool findPath(const Map &map, const VectorPosition &dst)
    {
        destination = dst;
        std::variant<Path<VectorPosition>, PathfindingFailureReason> path = pathFinder.findPath(locAndDir, destination, map);
        if (std::holds_alternative<Path<VectorPosition>>(path))
        {
            std::swap(this->path, std::get<Path<VectorPosition>>(path));
            return true;
        }
        else
        {
            return false;
        }
    }

    bool findPath(const Map &map)
    {
        return findPath(map, destination);
    }

    // 每一帧开始时更新路径
    void updatePath()
    {
        // 正常移动
        if (nextLocAndDir == locAndDir && !path.empty() && nextLocAndDir == path.back())
        {
            path.pop_back();
        }
        // 没有移动到预定位置
        else if (nextLocAndDir != VectorPosition(-1, -1, Direction::EAST) && nextLocAndDir != locAndDir)
        {
            LOGW(id, " 没有移动到预定位置, current pos: ", locAndDir, " next pos: ", nextLocAndDir);
        }
    }

    // 更新下一帧位置
    void updateNextPos()
    {
        if (!path.empty())
            // 寻路算法输出的路径是逆序存储的，以提高弹出效率
            nextLocAndDir = this->path.back();
        // 如果路径为空，则船舶下一帧不移动
        else
            nextLocAndDir = locAndDir;
    }

    // 临时移动到某个位置.
    void moveToTemporaryPosition(const VectorPosition &tempPos)
    {
        path.push_back(locAndDir);
        // 让船舶让路后多停一帧
        path.push_back(tempPos);
        path.push_back(tempPos);
        nextLocAndDir = tempPos;
    }

    // 移动到 nextLocAndDir
    std::string movetoNextPosture()
    {
        if (nextLocAndDir == SpatialUtils::moveForward(locAndDir)) {
            return ship();
        }
        else if (nextLocAndDir == SpatialUtils::clockwiseRotation(locAndDir)) {
            return rot(RotationDirection::Clockwise);
        }
        else if (nextLocAndDir == SpatialUtils::anticlockwiseRotation(locAndDir)) {
            return rot(RotationDirection::AntiClockwise);
        }
        else if (nextLocAndDir == locAndDir)
        {
            return "";
        }

        LOGW("船舶路径出错 ship ", id, " from: ", locAndDir, " to ", nextLocAndDir);
        return "";
    }
};