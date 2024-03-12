#pragma once
#include "utils.h"

class Goods {
private:
public:
    int id;
    int value;
    Point2d pos;
    int status; // 货物状态：0初始，1已分配，2已搬运，3已送达
    static int number;
    int initFrame = 0;  //起始帧数
    int TTL = 1000; //生存帧数，当TTL为-1时无法使用；INT_MAX时为不会过期
    // 构造函数等其他必要的函数
    Goods(Point2d pos, int value,int initFrame, int status=0) : id(number),pos(pos),value(value),initFrame(initFrame),status(status){number++;}

};