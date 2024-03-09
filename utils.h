#pragma once
#include <ostream>

#define DEBUG

const int MAPROWS = 200, MAPCOLS = 200;
const int ROBOTNUMS = 10, SHIPNUMS = 5, BERTHNUMS = 10;



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
    friend std::ostream& operator<<(std::ostream& os, const Point2d& point) // 重载输出运算符
    {
        os << "(" << point.x << "," << point.y << ")";
        return os;
    }
};

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

