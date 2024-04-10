#pragma once

#include <string>
#include <mutex>
#include "utils.h"
#include "log.h"
#include "map.h"
#include "pathFinder.h"
#include "assert.h"

namespace ShipStatusSpace{
    enum ShipStatus
    {
        IDLE = 0,            // 船处于空闲状态，等待新的任务分配
        MOVING_TO_BERTH, // 船正在移动至指定泊位。
        MOVING_TO_DELIVERY, // 船正在移动至指定货物位置。
        LOADING,       // 船正在装货物。
    };
}

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return (hash1 + 0x9e3779b9) ^ hash2;
    }
};

class SeaRoute
{
private:
    std::unordered_map<std::pair<VectorPosition, VectorPosition>, std::vector<VectorPosition>, pair_hash> seaRoutes;
    AStarPathfinder<VectorPosition, Map> pathFinder;

    SeaRoute() {}
    SeaRoute(const SeaRoute &) = delete;
    SeaRoute &operator=(const SeaRoute &) = delete;

public:
    static SeaRoute &getInstance()
    {
        static SeaRoute seaRout;
        return seaRout;
    }

    // 寻路并存储
    static bool findPath(const Map &map, const VectorPosition &start, const VectorPosition &destination)
    {
        static std::mutex seaRoutesMutex;
        if (getInstance().seaRoutes.find(std::make_pair(start, destination)) !=
            getInstance().seaRoutes.end())
            return true;

        // LOGI("Start: ",start," target: ", destination);
        std::variant<Path<VectorPosition>, PathfindingFailureReason> path = getInstance().pathFinder.findPath(start, destination, map);
        if (std::holds_alternative<Path<VectorPosition>>(path))
        {
            Path<VectorPosition> route = std::get<Path<VectorPosition>>(path);
            route.push_back(start);
            std::lock_guard<std::mutex> lock(seaRoutesMutex);
            getInstance().seaRoutes[std::make_pair(start, destination)] = route;
            return true;
        }
        else
        {
            return false;
        }
    }

    // 获取航线路径
    static std::vector<VectorPosition> getPath(const Map &map, const VectorPosition &start, const VectorPosition &destination)
    {
        std::vector<VectorPosition> path;
        if (getInstance().seaRoutes.find(std::make_pair(start, destination)) !=
            getInstance().seaRoutes.end())
            path = getInstance().seaRoutes[std::make_pair(start, destination)];
        return path;
    }
    
    // 获取航线长度
    static int getPathLength(const VectorPosition &start, const VectorPosition &destination)
    {
        int length = 0;
        if (getInstance().seaRoutes.find(std::make_pair(start, destination)) !=
            getInstance().seaRoutes.end())
            length = getInstance().seaRoutes[std::make_pair(start, destination)].size();
        return length;
    }
};


class Ship
{
public:
    int id;
    int goodsCount;           // 携带的货物数量
    VectorPosition locAndDir; //  表示船舶位置和方向
    int state;                // 0: 正常行驶状态, 1: 恢复状态, 2: 装载状态
    int berthId;              // 目标泊位 ID
    ShipStatusSpace::ShipStatus shipStatus;  // 船的状态
    int loadGoodValue;  // 船当前已有货物的价值
    // const int price = 8000;       // 购买价格
public:
    static int capacity;        // 船的容量
    int remainingTransportTime; // 船到目标泊位的剩余运行时间，在处理每一帧信息时维护
    VectorPosition destination;
    bool shouldDept;    //  判断当前是否需要使用dept命令
    int deliveryId; //分配的交货点id

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
          nextLocAndDir(-1, -1, Direction::EAST),
          shipStatus(ShipStatusSpace::ShipStatus::IDLE),
          shouldDept(false),
          loadGoodValue(0) {}

    // 比较优先级，true 表示本身的优先级大，false 表示 compareShip 优先级大
    bool comparePriority(Ship &compareShip){
        // 处于恢复状态的优先
        if (state != compareShip.state) {
            return state > compareShip.state;
        }
        // 路径短的优先
        else if (path.size() != compareShip.path.size()){
            return path.size() < compareShip.path.size();
        }
        // id小的优先
        else{
            return id < compareShip.id;
        } 
    }

    void resetDeptStatus(){
        shouldDept = false;
    }

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
        assert(state != 1);
#endif
        return "rot "s + std::to_string(id) + " " + std::to_string(static_cast<int>(rotDirection));
    }

    // 前进命令
    std::string ship()
    {
        using namespace std::string_literals;
#ifdef DEBUG
        // 船在正常行驶状态
        assert(state != 1);
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
        std::unordered_map<ShipStatusSpace::ShipStatus, std::string> shipStatusStr;
        shipStatusStr[ShipStatusSpace::ShipStatus::IDLE] = "空闲";
        shipStatusStr[ShipStatusSpace::ShipStatus::MOVING_TO_BERTH] = "前往泊位";
        shipStatusStr[ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY] = "前往交货点";
        shipStatusStr[ShipStatusSpace::ShipStatus::LOADING] = "装载状态";
        LOGI("船只", id, ",状态", state, ",路径长度：", path.size(), ",泊位id：", berthId, ",交货点id：", deliveryId, ",船舶状态：",shipStatusStr[shipStatus], ",目的地：", destination, ";");
        LOGI("当前位置：", locAndDir, ",下一帧位置：", nextLocAndDir, "路径长度：", path.size());
        LOGI("装货量：", capacity,", 装载价值: ", loadGoodValue,",剩余容量：", nowCapacity(), ",剩余容量比例：", nowCapacity() * 1.0 / capacity);
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
        LOGI("船舶寻路 from ", locAndDir, " to ", dst);
        destination = dst;
        Path<VectorPosition> route = SeaRoute::getPath(map, locAndDir, destination);
        if(!route.empty())
        {
            this->path = route;

            // std::vector<Point2d> path2D;
            // for(auto &point : this->path)
            //     path2D.push_back(point.pos);
            // LOGI(map.drawMap(nullptr, nullptr, &path2D, &locAndDir.pos, &destination.pos));

            return true;
        }
        // 如果没有寻找到预先存储的路径
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
            LOGW("Ship ",id, " 没有移动到预定位置, current pos: ", locAndDir, " next pos: ", nextLocAndDir);
        }
    }

    // 更新下一帧位置
    void updateNextPos()
    {
        // 正常航行状态才会前进
        if (!path.empty() && state != 1)
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
        // LOGI("下一帧位置：", nextLocAndDir);
        if (nextLocAndDir == SpatialUtils::moveForward(locAndDir))
        {
            return ship();
        }
        else if (nextLocAndDir == SpatialUtils::clockwiseRotation(locAndDir))
        {
            return rot(RotationDirection::Clockwise);
        }
        else if (nextLocAndDir == SpatialUtils::anticlockwiseRotation(locAndDir))
        {
            return rot(RotationDirection::AntiClockwise);
        }
        else if (nextLocAndDir == locAndDir)
        {
            return "";
        }

        LOGW("船舶路径出错 ship ", id, " from: ", locAndDir, " to ", nextLocAndDir);
        return "";
    }

    // 判断是否到达目的地
    bool reachDestination(){
        if (destination.pos == locAndDir.pos) return true;
        else return false;
    }

    // 判断是否到达泊位
    // todo 需要判断是否进入泊位范围内
    bool reachBerth(){
        if (reachDestination() && shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_BERTH) return true;
        else return false;
    }
    
    // 判断是否到达交货点
    bool reachDelivery(){
        if (reachDestination() && shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY){
            return true;
        }
        else return false;
    }

    // 判断船是否空闲
    bool isIdle(){
        return shipStatus == ShipStatusSpace::ShipStatus::IDLE;
    }

    // 判断船是否前往交货点
    bool isMoveToDelivery(){
        return shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY;
    }

    // 判断船是否前往泊位
    bool isMoveToBerth(){
        return shipStatus == ShipStatusSpace::ShipStatus::MOVING_TO_BERTH;
    }

    // 判断船是否在装载
    bool isLoading(){
        return shipStatus == ShipStatusSpace::ShipStatus::LOADING;
    }

    // 装货状态处理
    void updateLoadStatus(){
        LOGI("船",id,",装货状态");
        shipStatus = ShipStatusSpace::ShipStatus::LOADING;
        destination = VectorPosition({-1,-1}, Direction::EAST);
        // 路径清空
        path.clear();
    }

    // 前往泊位状态处理
    void updateMoveToBerthStatus(BerthID berthId, VectorPosition destination){
        LOGI("船",id,",前往泊位状态");
        shipStatus = ShipStatusSpace::ShipStatus::MOVING_TO_BERTH;
        // 交货点id复原
        this->deliveryId = -1;
        this->berthId = berthId;
        //todo 方向如何确定
        this->destination = destination;
        // 路径清空
        path.clear();
    }

    // 前往交货点状态处理
    void updateMoveToDeliveryStatus(int deliveryId, VectorPosition destination){
        LOGI("船",id,",前往交货点状态");
        shipStatus = ShipStatusSpace::ShipStatus::MOVING_TO_DELIVERY;
        this->deliveryId = deliveryId;
        // todo方向如何确定
        this->destination = destination;
        // 路径清空
        path.clear();
    }

    // 判断目的地位置是否合法
    bool isDestinationValid(){
        if (destination.pos.x == -1 || destination.pos.y == -1) return false;
        else return true;
    }

    friend std::ostream &operator<<(std::ostream &os, const Ship &ship)
    {
        os << "Ship id: " << ship.id << " locAndDir: " << ship.locAndDir << " nextlocAndDir: " << ship.nextLocAndDir << " dst: " << ship.destination << " path: " << ship.path.size() << ", ";
        for (int i = (int)ship.path.size() - 1; i >= std::max(0, (int)ship.path.size() - 5); --i)
        {
            os << ship.path[i];
        }
        return os;
    }
};




