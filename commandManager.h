#pragma once

#include <vector>
#include <string>
#include <iostream>
#include "robot.h"
#include "ship.h"
#include "goods.h"
// 这个类负责收集机器人和船只的指令，并最终统一输出。
class CommandManager
{
private:
    std::vector<std::string> robotCommands; // 存储机器人指令
    std::vector<std::string> shipCommands;  // 存储船只指令

public:
    void addRobotCommand(const std::string &command)
    {
        robotCommands.emplace_back(command);
    }
    void addShipCommand(const std::string &command)
    {
        shipCommands.emplace_back(command);
    }
    void outputCommands() const
    {
        // 输出所有机器人指令
        for (const auto &command : robotCommands)
            std::cout << command << std::endl;
        // 输出所有船舶指令
        for (const auto &command : shipCommands)
            std::cout << command << std::endl;
        // 刷新输出缓冲区
        std::cout << std::flush;
    }
    void clearCommands()
    {
        robotCommands.clear();
        shipCommands.clear();
    }
};