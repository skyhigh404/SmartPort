#include "scheduler.h"

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleRobots(std::vector<Robot>& robots, const Map& map, std::vector<Goods>& goods, CommandManager& commandManager)
{
    return std::vector<std::pair<int, Action>>();
}

std::vector<std::pair<int, Action>>  SimpleTransportStrategy::scheduleShips(std::vector<Ship>& ships, std::vector<Berth>& berths, CommandManager& commandManager)
{
    return std::vector<std::pair<int, Action>>();
}