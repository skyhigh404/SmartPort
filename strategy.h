#pragma once
#include "goods.h"
#include "map.h"


class TransportStrategy {
public:
    virtual void transport(Goods& goods, const Map& map) = 0;
};

class SimpleTransportStrategy : public TransportStrategy {
public:
    void transport(Goods& goods, const Map& map) override {
        // 简单搬运算法实现
    }
};

class EfficientTransportStrategy : public TransportStrategy {
public:
    void transport(Goods& goods, const Map& map) override {
        // 高效搬运算法实现
    }
};
