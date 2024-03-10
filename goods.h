#pragma once
#include "utils.h"

class Goods {
private:
public:
    int id;
    int value;
    Point2d pos;
    int status; // 货物状态：0初始，1搬运中，2已送达
    static int number;
    // 构造函数等其他必要的函数
    Goods(Point2d pos, int value, int status=0) : id(number),pos(pos),value(value),status(status){number++;}
};