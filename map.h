#pragma once

#include <vector>
namespace MapItemSpace
{
    enum MapItem
    {
        SPACE = 0,
        SEA,
        OBSTACLE,
        ROBOT,
        BERTH,
        ERROR = -1,
    };
}

class Map
{
    // 地图坐标系原点在左上角，往下为 X 轴正方向，往右为 Y 轴正方向
public:
    int rows, cols;
    std::vector<std::vector<MapItemSpace::MapItem>> grid;

public:
    Map(int rows, int cols)
        : rows(rows),
          cols(cols),
          grid(std::vector(rows, std::vector<MapItemSpace::MapItem>(cols, MapItemSpace::SPACE)))
    {
    }

    // 设置地图上某个位置的值
    void setCell(int x, int y, MapItemSpace::MapItem value)
    {
        if (x >= 0 && x < rows && y >= 0 && y < cols)
        {
            grid[x][y] = value;
        }
    }

    // 获取地图上某个位置的值
    int getCell(int x, int y) const
    {
        if (x >= 0 && x < rows && y >= 0 && y < cols)
        {
            return grid[x][y];
        }
        return MapItemSpace::MapItem::ERROR; // 返回-1表示越界或者错误
    }

    // // 打印地图的简单方法，用于调试
    // void printMap() const
    // {
    //     for (const auto &row : grid)
    //     {
    //         for (int cell : row)
    //         {
    //             std::cout << (cell == 0 ? "." : "#") << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    // }

    // 其他地图管理相关的方法，如路径查找、障碍物管理等
};
