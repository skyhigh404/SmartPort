#pragma once
#include "utils.h"

class Goods
{
public:
    static int count;

public:
    int id;
    int value;
    Point2d pos;

public:
    int status;        // 货物状态：0初始，1已分配，2已搬运，3已送达
    int initFrame = 0; // 起始帧数
    int TTL;           // 剩余生存帧数，当TTL为 -1 时无法使用；INT_MAX时为不会过期

    std::vector<std::pair<int, int>> distsToBerths; // 存储货物到港口的距离，第一个是泊位 ID，第二个是距离，应该为降序存储

    Goods(Point2d pos, int value, int initFrame, int status = 0)
        : id(count),
          value(value),
          pos(pos),
          status(status),
          initFrame(initFrame),
          TTL(1000) { count++; }
};