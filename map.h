#pragma once

#include <vector>

class Map {
private:
    // 地图的宽度和高度
    int width, height;
    // 存储地图信息的二维数组，这里简单使用int类型，可以根据需要定义更复杂的类型
    // 例如，0表示空地，1表示障碍物
    std::vector<std::vector<int>> grid;

public:
    // 构造函数，初始化地图的大小和内容
    Map(int width, int height) : width(width), height(height), grid(height, std::vector<int>(width, 0)) {}

    // 设置地图上某个位置的值
    void setCell(int x, int y, int value) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            grid[y][x] = value;
        }
    }

    // 获取地图上某个位置的值
    int getCell(int x, int y) const {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            return grid[y][x];
        }
        return -1; // 返回-1表示越界或者错误
    }

    // 打印地图的简单方法，用于调试
    void printMap() const {
        for (const auto& row : grid) {
            for (int cell : row) {
                std::cout << (cell == 0 ? "." : "#") << " ";
            }
            std::cout << std::endl;
        }
    }

    // 其他地图管理相关的方法，如路径查找、障碍物管理等
};
