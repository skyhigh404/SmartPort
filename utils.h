#pragma once
#include <ostream>
#include <cmath>
#include <tuple>

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
    Point2d operator+(const Point2d& other) const // 重载+向量加法运算符
    {
        return Point2d(x + other.x, y + other.y);
    }
    friend std::ostream& operator<<(std::ostream& os, const Point2d& point) // 重载输出运算符
    {
        os << "(" << point.x << "," << point.y << ")";
        return os;
    }
    static inline int calculateManhattanDistance(const Point2d& p1, const Point2d& p2) {
        return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
    }
    static inline float calculateEuclideanDistance(const Point2d& p1, const Point2d& p2) {
        std::sqrt((double)(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2)));
    }

    float operator*(const Point2d& other) const {   // 重载*运算符以实现点积
        return this->x * other.x + this->y * other.y;
    }

    float magnitude() const {   // 提供一个成员函数来计算模长
        return std::sqrt(this->x * this->x + this->y * this->y);
    }
    
    float cosineTo(const Point2d& point1, const Point2d& point2) const {    // 计算以当前点到另外两点形成的向量夹角的余弦值
        // 计算向量
        Point2d vectorToPoint1 = {point1.x - this->x, point1.y - this->y};
        Point2d vectorToPoint2 = {point2.x - this->x, point2.y - this->y};

        // 计算点积
        float dotProduct = vectorToPoint1 * vectorToPoint2;

        // 计算各向量的模长
        float magnitudeToPoint1 = vectorToPoint1.magnitude();
        float magnitudeToPoint2 = vectorToPoint2.magnitude();

        // 计算余弦值
        return dotProduct / (magnitudeToPoint1 * magnitudeToPoint2);
    }
};

inline bool operator < (const Point2d &a, const Point2d &b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <> struct hash<Point2d> {
  std::size_t operator()(const Point2d& id) const noexcept {
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

    Vec2f(const Vec2f& other) : x(other.x), y(other.y) {} // 拷贝构造函数
    Vec2f(Vec2f&& other) noexcept : x(std::exchange(other.x, 0)), y(std::exchange(other.y, 0)) {} // 移动构造函数

    Vec2f& operator=(const Vec2f& other) // 拷贝赋值运算符
    {
        if (this != &other)
        {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    Vec2f& operator=(Vec2f&& other) noexcept // 移动赋值运算符
    {
        if (this != &other)
        {
            x = std::exchange(other.x, 0);
            y = std::exchange(other.y, 0);
        }
        return *this;
    }

    bool operator==(const Vec2f& other) const // 重载==比较运算符
    {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const Vec2f& other) const // 重载!=比较运算符
    {
        return !(*this == other);
    }

    Vec2f operator+(const Vec2f& other) const // 重载+向量加法运算符
    {
        return Vec2f(x + other.x, y + other.y);
    }

    Vec2f& operator+=(const Vec2f& other) // 重载+=向量加法赋值运算符
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2f operator-(const Vec2f& other) const // 重载-向量减法运算符
    {
        return Vec2f(x - other.x, y - other.y);
    }

    Vec2f& operator-=(const Vec2f& other) // 重载-=向量减法赋值运算符
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2f operator*(float scalar) const // 重载*向量与标量乘法运算符
    {
        return Vec2f(x * scalar, y * scalar);
    }

    Vec2f& operator*=(float scalar) // 重载*=向量与标量乘法赋值运算符
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec2f& vec) // 重载输出运算符
    {
        os << "(" << vec.x << "," << vec.y << ")";
        return os;
    }
};

