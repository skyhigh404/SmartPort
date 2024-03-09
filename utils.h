#pragma once

#define DEBUG

const int MAPROWS = 200, MAPCOLS = 200;
const int ROBOTNUMS = 10, SHIPNUMS = 5, BERTHNUMS = 10;
struct Point2d
{
    int x,y;
    
    Point2d() : x(0), y(0){}
    Point2d(int x, int y) : x(x), y(y){}
};

struct Vec2f
{
    float x,y;

    Vec2f() : x(0), y(0){}
    Vec2f(int x, int y) : x(x), y(y){}
};

