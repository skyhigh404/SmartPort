#pragma once

#include <vector>
#include <string>
#include <iostream>
#include "robot.h"
#include "ship.h"
#include "goods.h"
#include "log.h"
#include <sstream>
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
        std::ostringstream stream;
        // 输出所有机器人指令
        for (const auto &command : robotCommands)
            stream << command << "\n";
        // 输出所有船舶指令
        for (const auto &command : shipCommands)
            stream << command << "\n";
        stream << "OK\n";
        // 刷新输出缓冲区
        std::cout << stream.str() << std::flush;
    }
    void clearCommands()
    {
        robotCommands.clear();
        shipCommands.clear();
    }
};