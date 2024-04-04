#pragma once
#include <ostream>
#include <cmath>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <climits>

#define DEBUG

const int MAPROWS = 200, MAPCOLS = 200;
const int ROBOTNUMS = 10, SHIPNUMS = 5, BERTHNUMS = 10;

using BerthID = int;
using RobotID = int;
using ShipID = int;
using GoodsID = int;

struct Point2d
{
    int x, y;

    Point2d() : x(-1), y(-1) {}
    Point2d(int x, int y) : x(x), y(y) {}
    Point2d(const Point2d &other) : x(other.x), y(other.y) {}
    Point2d(Point2d &&other) noexcept : x(std::exchange(other.x, -1)), y(std::exchange(other.y, -1)) {}
    Point2d &operator=(const Point2d &other) // 拷贝赋值运算符
    {
        if (this != &other)
        {
            x = other.x;
            y = other.y;
        }
        return *this;
    }
    Point2d &operator=(Point2d &&other) noexcept // 移动赋值运算符
    {
        if (this != &other)
        {
            x = std::exchange(other.x, -1);
            y = std::exchange(other.y, -1);
        }
        return *this;
    }
    bool operator==(const Point2d &other) const // 重载==比较运算符
    {
        return (x == other.x && y == other.y);
    }
    bool operator!=(const Point2d &other) const // 重载!=比较运算符
    {
        return !(*this == other);
    }
    Point2d operator+(const Point2d &other) const // 重载+向量加法运算符
    {
        return Point2d(x + other.x, y + other.y);
    }
    friend std::ostream &operator<<(std::ostream &os, const Point2d &point) // 重载输出运算符
    {
        os << "(" << point.x << "," << point.y << ")";
        return os;
    }
    static inline int calculateManhattanDistance(const Point2d &p1, const Point2d &p2)
    {
        return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
    }
    static inline float calculateEuclideanDistance(const Point2d &p1, const Point2d &p2)
    {
        return std::sqrt((float)(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2)));
    }
    static bool isIN(Point2d pos, std::vector<Point2d> poss)
    {
        for (const auto &p : poss)
            if (p == pos)
                return true;
        return false;
    }

    int operator*(const Point2d &other) const
    { // 重载*运算符以实现点积
        return this->x * other.x + this->y * other.y;
    }
};

inline bool operator<(const Point2d &a, const Point2d &b)
{
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}


enum class Direction
{
    EAST = 0, // 右
    WEST,     // 左
    NORTH,    // 上
    SOUTH     // 下
};

// 有朝向的物体
struct VectorPosition
{
    Point2d pos;         // 位置坐标
    Direction direction; // 朝向

    VectorPosition() : direction(Direction::EAST) {}
    VectorPosition(Point2d _pos, Direction _d) : pos(_pos), direction(_d) {}
    VectorPosition(const VectorPosition &other) : pos(other.pos), direction(other.direction) {}
    VectorPosition(VectorPosition &&other) noexcept
        : pos(std::exchange(other.pos, {-1, -1})), direction(std::exchange(other.direction, Direction::EAST)) {}
    VectorPosition &operator=(const VectorPosition &other) // 拷贝赋值运算符
    {
        if (this != &other)
        {
            pos.x = other.pos.x;
            pos.y = other.pos.y;
            direction = other.direction;
        }
        return *this;
    }
    VectorPosition &operator=(VectorPosition &&other) noexcept // 移动赋值运算符
    {
        if (this != &other)
        {
            pos.x = std::exchange(other.pos.x, -1);
            pos.y = std::exchange(other.pos.y, -1);
            direction = std::exchange(other.direction, Direction::EAST);
        }
        return *this;
    }
    bool operator==(const VectorPosition &other) const {
        return other.pos == other.pos && direction == other.direction;
    }
    bool operator!=(const VectorPosition &other) const {
        return !(*this == other);
    }
    friend std::ostream &operator<<(std::ostream &os, const VectorPosition &vp) {
        os << "(" << vp.pos.x << "," << vp.pos.y << "," << static_cast<int>(vp.direction) << ")";
        return os;
    }

    // 旋转多少次（每次 90 度）可以达到目标方向，逆时针方向为正，取值范围 (-1, 2]
    static inline int minimalRotationStep(Direction begin, Direction end) {
        static const std::array<std::array<int, 4>, 4> rotationStepsTable  = {{
            {0, 2, 1, -1},  // Comparisons for 0 
            {2, 0, -1, 1},  // Comparisons for 1
            {-1, 1, 0, 2},  // Comparisons for 2
            {1, -1, 2, 0}   // Comparisons for 3
        }};

    return rotationStepsTable[static_cast<int>(begin)][static_cast<int>(end)];
    }
};

namespace std
{
    template <>
    struct hash<Point2d>
    {
        std::size_t operator()(const Point2d &id) const noexcept
        {
            return std::hash<int>()(id.x ^ (id.y << 16));
        }
    };

    template <>
    struct hash<VectorPosition>
    {
        std::size_t operator()(const VectorPosition &id) const noexcept
        {
            return std::hash<int>()(id.pos.x ^ (id.pos.y << 16) ^ (static_cast<int>(id.direction) << 24));
        }
    };
}

class SpatialUtils
{
public:
    // 给定船的核心点和朝向，返回该船舶占用空间的矩形的左上角和右下角坐标，船舶大小为2*3
    static std::pair<Point2d, Point2d> getShipOccupancyRect(const VectorPosition &e)
    {
        // 核心点坐标和朝向到船占据的矩形的映射表
        static const std::array<std::pair<Point2d, Point2d>, 4> occupancyOffsetTable = {
            {{{0, 0}, {1, 2}},   // EAST: 左上角偏移(0,0)，右下角偏移(1,2)
             {{-1, -2}, {0, 0}}, // WEST: 左上角偏移(-1,-2)，右下角偏移(0,0)
             {{-2, 0}, {0, 1}},  // NORTH: 左上角偏移(-2,0)，右下角偏移(0,1)
             {{0, -1}, {2, 0}}}  // SOUTH: 左上角偏移(0,-1)，右下角偏移(2,0)
        };
        const auto &offsets = occupancyOffsetTable[static_cast<int>(e.direction)];
        Point2d topLeft = {e.pos.x + offsets.first.x, e.pos.y + offsets.first.y};
        Point2d bottomRight = {e.pos.x + offsets.second.x, e.pos.y + offsets.second.y};
        return {topLeft, bottomRight};
    }
    // 返回前进一格后的核心点位置和方向
    static VectorPosition moveForward(const VectorPosition &vp)
    {
        // 前进一格后的位置
        static const std::array<Point2d, 4> moveOneStep = {{
            {0, 1},  // 右
            {0, -1}, // 左
            {-1, 0}, // 上
            {1, 0}   // 下
        }};
        return {vp.pos + moveOneStep[static_cast<int>(vp.direction)], vp.direction};
    }

    // 返回逆时针旋转一次后的核心点位置和方向
    static VectorPosition anticlockwiseRotation(const VectorPosition &vp)
    {
        // 逆时针旋转一次后的方向
        static const std::array<Direction, 4> directionChange = {
            Direction::NORTH, // 右旋转到上
            Direction::SOUTH, // 左旋转到下
            Direction::WEST,  // 上旋转到左
            Direction::EAST   // 下旋转到右
        };
        // 逆时针旋转一次后核心点的位置
        static const std::array<Point2d, 4> corePositionChange = {{
            {1, 1},   // 右
            {-1, -1}, // 左
            {-1, 1},  // 上
            {1, -1}   // 下
        }};
        return {vp.pos + corePositionChange[static_cast<int>(vp.direction)], directionChange[static_cast<int>(vp.direction)]};
    }

    // 返回顺时针旋转一次后的核心点位置和方向
    static inline VectorPosition clockwiseRotation(const VectorPosition &vp)
    {
        // 顺时针旋转一次后的方向
        static const std::array<Direction, 4> directionChange = {
            Direction::SOUTH, // 右旋转到下
            Direction::NORTH, // 左旋转到上
            Direction::EAST,  // 上旋转到右
            Direction::WEST   // 下旋转到左
        };
        // 顺时针旋转一次后核心点的位置
        static const std::array<Point2d, 4> corePositionChange = {{
            {0, 2},  // 右
            {0, -2}, // 左
            {-2, 0}, // 上
            {2, 0}   // 下
        }};
        return {vp.pos + corePositionChange[static_cast<int>(vp.direction)], directionChange[static_cast<int>(vp.direction)]};
    }
};

struct Vec2f
{
    float x, y;

    Vec2f() : x(0), y(0) {}
    Vec2f(float x, float y) : x(x), y(y) {}
    // 根据两个点构造向量
    Vec2f(const Point2d &from, const Point2d &to)
        : x(static_cast<float>(to.x - from.x)), y(static_cast<float>(to.y - from.y)) {}

    Vec2f(const Vec2f &other) : x(other.x), y(other.y) {}                                         // 拷贝构造函数
    Vec2f(Vec2f &&other) noexcept : x(std::exchange(other.x, 0)), y(std::exchange(other.y, 0)) {} // 移动构造函数

    Vec2f &operator=(const Vec2f &other) // 拷贝赋值运算符
    {
        if (this != &other)
        {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    Vec2f &operator=(Vec2f &&other) noexcept // 移动赋值运算符
    {
        if (this != &other)
        {
            x = std::exchange(other.x, 0);
            y = std::exchange(other.y, 0);
        }
        return *this;
    }

    bool operator==(const Vec2f &other) const // 重载==比较运算符
    {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const Vec2f &other) const // 重载!=比较运算符
    {
        return !(*this == other);
    }

    Vec2f operator+(const Vec2f &other) const // 重载+向量加法运算符
    {
        return Vec2f(x + other.x, y + other.y);
    }

    Vec2f &operator+=(const Vec2f &other) // 重载+=向量加法赋值运算符
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2f operator-(const Vec2f &other) const // 重载-向量减法运算符
    {
        return Vec2f(x - other.x, y - other.y);
    }

    Vec2f &operator-=(const Vec2f &other) // 重载-=向量减法赋值运算符
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2f operator*(float scalar) const // 重载*向量与标量乘法运算符
    {
        return Vec2f(x * scalar, y * scalar);
    }

    float operator*(const Vec2f &other) const
    {
        return x * other.x + y * other.y;
    }

    Vec2f &operator*=(float scalar) // 重载*=向量与标量乘法赋值运算符
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    friend std::ostream &operator<<(std::ostream &os, const Vec2f &vec) // 重载输出运算符
    {
        os << "(" << vec.x << "," << vec.y << ")";
        return os;
    }

    float magnitude() const
    { // 提供一个成员函数来计算模长
        return std::sqrt(this->x * this->x + this->y * this->y);
    }

    static float cosineOf2Vec(const Vec2f &vec1, const Vec2f &vec2)
    { // 计算以当前点到另外两点形成的向量夹角的余弦值
        // 计算余弦值
        return vec1 * vec2 / (vec1.magnitude() * vec2.magnitude());
    }
};

enum ActionType
{
    MOVE_TO_POSITION,
    PICK_UP_GOODS,
    DROP_OFF_GOODS,
    MOVE_TO_BERTH,
    DEPART_BERTH,
    FIND_PATH,
    FAIL,
    CONTINUE
};

struct Action
{
    ActionType type;
    Point2d destination; // 用于移动
    int targetId;        // 用于标识具体的货物或泊位，根据上下文决定其含义
    Action() {}
    Action(ActionType type, Point2d destination, int targetId) : type(type), destination(destination), targetId(targetId) {}
    Action(ActionType type) : type(type), destination(Point2d(-1, -1)), targetId(-1) {}
};
namespace RobotActionSpace
{
    // 指示机器人下一步应采取的行动
    enum RobotActionType
    {
        // 重新明确调度器指示的机器人下一步应采取的行动，
        MOVE_TO_BERTH,  // 移动到泊位目标位置，需要提供目标位置和泊位 ID，机器人之后进行寻路
        MOVE_TO_GOOD,   // 移动到货物目标位置，需要提供目标位置和货物 ID，机器人之后进行寻路
        MOVE_TO_TARGET, // 移动到目标位置，需要提供目标位置和，机器人之后进行寻路（留做扩展用）
        PERFORM_TASK,   // 执行特定任务（如装卸），系统在开始时自动执行判断，调度函数无需赋值为此项
        FAILURE,        // 表示失败，机器人或系统需要重新评估情况
        WAIT,           // 等待进一步指令，可以用于当机器人需要暂时停下来等待新的任务分配
        CONTINUE        // 继续当前操作
    };

    struct RobotAction
    {
        RobotActionType type; // 行动类型
        Point2d destination;  // 用于MOVE_TO_TARGET的目的地坐标
        int targetId;         // 标识具体任务或对象的ID，例如货物ID或泊位ID

        RobotAction(RobotActionType type) : type(type), destination(Point2d(-1, -1)), targetId(-1) {}
        RobotAction(RobotActionType type, Point2d destination)
            : type(type), destination(destination), targetId(-1) {}
        RobotAction(RobotActionType type, Point2d destination, int targetId)
            : type(type), destination(destination), targetId(targetId) {}
    };
}

namespace ShipActionSpace
{
    enum ShipActionType
    {
        MOVE_TO_BERTH,
        DEPART_BERTH,
        CONTINUE
    };

    struct ShipAction
    {
        ShipActionType type;
        int targetId; // 用于标识具体的货物或泊位，根据上下文决定其含义
        ShipAction(ShipActionType type) : type(type), targetId(-1) {}
        ShipAction(ShipActionType type, int targetId)
            : type(type), targetId(targetId) {}
        ShipAction() : targetId(-1) {}
    };
}