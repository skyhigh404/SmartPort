#pragma once
#include "utils.h"

class Goods {
public:
    int value;
    Point2d pos;
    // 构造函数等其他必要的函数
    Goods(Point2d pos, int value) : pos(pos),value(value){}
};