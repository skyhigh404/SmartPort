#pragma once
#include "goods.h"
#include "map.h"
#include "strategy.h"

class Robot {
private:
    TransportStrategy* strategy;
public:
    void setTransportStrategy(TransportStrategy* strategy) {
        this->strategy = strategy;
    }

    void transportGoods(Goods& goods, const Map& map) {
        strategy->transport(goods, map);
    }
};
