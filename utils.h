#pragma once
#include <ostream>
#include <cmath>
#include <tuple>
#include <unordered_map>

#define DEBUG

const int MAPROWS = 200, MAPCOLS = 200;
const int ROBOTNUMS = 10, SHIPNUMS = 5, BERTHNUMS = 10;

using BerthID = int;

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
        return std::sqrt((double)(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2)));
    }
    static bool isIN(Point2d pos, std::vector<Point2d> poss)
    {
        for(const auto &p : poss)
            if(p == pos)
                return true;
        return false;
    }

    float operator*(const Point2d &other) const
    { // 重载*运算符以实现点积
        return this->x * other.x + this->y * other.y;
    }
};

inline bool operator<(const Point2d &a, const Point2d &b)
{
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

namespace std
{
    /* implement hash function so we can put GridLocation into an unordered_set */
    template <>
    struct hash<Point2d>
    {
        std::size_t operator()(const Point2d &id) const noexcept
        {
            // NOTE: better to use something like boost hash_combine
            return std::hash<int>()(id.x ^ (id.y << 16));
        }
    };
}

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

class Cost
{
public:
    // Costs are represented as 24:8 fixed point values.
    static const int FP_MULT = (1 << 8);

    static Cost max()
    {
        Cost c;
        // c.value = std::numeric_limits<int>::max() / FP_MULT;
        c.value = 10000000 / FP_MULT;
        return c;
    }

    Cost() : value(0) {}
    Cost(const double c) : value(FP_MULT * c) {}
    Cost(const float c) : value(FP_MULT * c) {}
    Cost(const int c) : value(FP_MULT * c) {}

    double as_double() const { return value / static_cast<double>(FP_MULT); }
    float as_float() const { return value / static_cast<float>(FP_MULT); }
    int as_int() const { return value / FP_MULT; }

    bool operator<(const Cost &rhs) const { return value < rhs.value; }
    bool operator<=(const Cost &rhs) const { return value <= rhs.value; }
    bool operator==(const Cost &rhs) const { return value == rhs.value; }
    bool operator!=(const Cost &rhs) const { return value != rhs.value; }
    bool operator>(const Cost &rhs) const { return value > rhs.value; }
    bool operator>=(const Cost &rhs) const { return value >= rhs.value; }

    friend Cost operator+(const Cost &lhs, const Cost &rhs);
    friend Cost operator-(const Cost &lhs, const Cost &rhs);
    friend Cost operator*(const double d, const Cost &rhs);

private:
    int value = 0;
};

inline Cost operator+(const Cost &lhs, const Cost &rhs)
{
    Cost c;
    c.value = lhs.value + rhs.value;
    return c;
}

inline Cost operator-(const Cost &lhs, const Cost &rhs)
{
    Cost c;
    c.value = lhs.value - rhs.value;
    return c;
}

inline Cost operator*(const double d, const Cost &rhs)
{
    Cost c;
    c.value = d * rhs.value;
    return c;
}

template <class KEY, class VAL>
class HashMap
{
public:
    inline void put(const KEY &key, const VAL &val) { map[key] = val; }

    inline VAL at(const KEY &key)
    {
        if (map.find(key) == map.end())
        {
            return Cost::max();
        }
        return map[key];
    }

    bool contains(const KEY &key) const { return map.find(key) != map.end(); }
    std::size_t size() const { return map.size(); }
    void clear() { map.clear(); }

private:
    std::unordered_map<KEY, VAL> map;
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
    Point2d desination; // 用于移动
    int targetId;     // 用于标识具体的货物或泊位，根据上下文决定其含义
    Action(){}
    Action(ActionType type, Point2d desination, int targetId) : type(type), desination(desination), targetId(targetId){}
    Action(ActionType type) : type(type), desination(Point2d(-1,-1)), targetId(-1){}
};
