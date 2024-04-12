#pragma once
#include "map.h"
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <sstream>

struct Params
{
    // 注意命名规范，防止不同的子类超参互相冲突
    // robot 参数
    // 例如，float greedyRobotScheduleTTLProfitWeight
    // 泊位聚类超参
    int CLUSTERNUMS = 4;                    // 泊位的聚类类别数目
    int FINAL_FRAME = 14000;    // 终局帧数

    // 机器人调度超参
    float robot2goodWeight = 1;                 // 机器人到货物的距离权重
    float good2berthWeight = 1;                 // 货物到泊位的距离权重
    float TTL_ProfitWeight = 1.5;
    int TTL_Bound = 500;
    bool PartitionScheduling = true;        // 是否分区调度
    int startPartitionScheduling = 0;       // 开始分区调度的机器人数，0:一开局就分区调度，maxRobotNum:机器人全买完才开始分区调度
    bool DynamicPartitionScheduling = true; // 是否动态分区调度
    std::vector<int> ASSIGNBOUND;           // 手动设置各个类分配的机器人数目，总数目应等于机器人数目
    float robotReleaseBound = 0.8;          //低于平均泊位价值的比值时，释放机器人去其他泊位
    int DynamicSchedulingInterval = 200;    // 动态调度间隔
    bool FinalgameScheduling = true;        // 是否终局调度
    
    // 购买策略超参
    int maxRobotNum = 15;                   // 最多购买机器人数目
    int maxShipNum = 2;                     // 最多购买船只数目
    std::vector<std::vector<int>> robotPurchaseAssign = {{8, 100}, {1, 4}, {1, 4}};
    // std::vector<std::vector<int>> shipPurchaseAssign = {{4, 4, 4, 5, 6, 7, 8, 9, 10}, {1, 2, 3, 4, 5, 6}, {1, 2, 3, 4, 5, 6}};
    std::vector<std::vector<int>> shipPurchaseAssign = {{1, 4, 10}, {1, 2, 0}, {1, 2, 0}};
    int timeToBuyShip = 50;                 // 开始购买第二艘船的时间
    int startNum = 1;                       // 最初的数目（机器人、轮船）
    float landDistanceWeight = 10.0;        // 对泊位价值评估时的陆地访问距离权重
    float deliveryDistanceWeight = 10.0;    // 对泊位价值评估时的交货点访问距离权重
    bool CentralizedTransportation = false;  // 游戏开局是否集中调度
    bool robotFirst = true;                 // 先买机器人还是先买船，true为机器人、false为船

    //  船调度超参
    float ABLE_DEPART_SCALE = 0.15;         //可以去虚拟点的剩余容量比例
    int MAX_SHIP_NUM = 1;                   // 一个泊位最多几艘船
    int TIME_TO_WAIT = 100;                 //等待有货的时间段
    int CAPACITY_GAP = 10;                  // 泊位溢出货物量和船的容量差
    int SHIP_WAIT_TIME_LIMIT = 10;           //船在泊位上等待货物的时间
    int GOOD_DISTANCE_LIMIT = 100;  // 只考虑距离泊位[0, GOOD_DISTNACE_LIMIT]内的货物价值
    int DELIVERY_VALUE_LIMIE = 1000;   // 当船运输价值 > DELIVERY_VALUE_LIMIE 时才能去虚拟点（终局时刻例外）

    int EARLY_DELIVERT_FRAME_LIMIT = 2000;  // 当前帧数< EARLY_DELIVERT_FRAME_LIMIT时，船赚到EARLY_DELIVERY_VALUE_LIMIT - CURRERY_MONEY钱就去虚拟点
    int EARLY_DELIVERY_VALUE_LIMIT = 2000; 
    

    
    int SHIP_STILL_FRAMES_LIMIE = 5;    // 船阻塞帧数限制

    Params(MapFlag mapFalg)
    {
        if (mapFalg == MapFlag::MAP1)
        {
        }
        else if (mapFalg == MapFlag::MAP2)
        {
        }
        else if (mapFalg == MapFlag::MAP3)
        {
        }
        else if (mapFalg == MapFlag::NORMAL)
        {
        }
        else
            LOGE("初始化参数为失败");
    }
};

// 参数读取类
class ParamReader {
public:
    ParamReader() {}

    bool readParams(const std::string filename) {
        LOGI("读取文件中:", filename);
        std::ifstream file(filename);
        if (!file.is_open()) {
            LOGE("无法打开文件");
            return false;
        }

        std::string line;
        while (getline(file, line)) {
            std::istringstream iss(line);
            std::string key;
            std::string value;

            if (!(iss >> key >> value)) {
                LOGI("读取参数时报错");
                continue; // 跳过有误的行
            }

            params_[key] = value;
        }
        LOGI("读取超参数成功");
        file.close();
        return true;
    }
    
    // 设置地图的超参数类
    void setParams(Params &param){
        auto setIntParam = [this](int& param, const std::string& key) {
            if (params_.find(key) != params_.end()) {
                param = std::stoi(params_[key]);
            }
        };

        auto setFloatParam = [this](float& param, const std::string& key) {
            if (params_.find(key) != params_.end()) {
                param = std::stof(params_[key]);
            }
        };

        auto setBoolParam = [this](bool& param, const std::string& key) {
            if (params_.find(key) != params_.end()) {
                param = params_[key] == "1";
            }
        };

        setIntParam(param.FINAL_FRAME, "FINAL_FRAME");
        setIntParam(param.maxRobotNum, "maxRobotNum");
        setFloatParam(param.landDistanceWeight, "landDistanceWeight");
        setFloatParam(param.deliveryDistanceWeight, "deliveryDistanceWeight");
        setBoolParam(param.CentralizedTransportation, "CentralizedTransportation");
        setBoolParam(param.robotFirst, "robotFirst");
        setBoolParam(param.PartitionScheduling, "PartitionScheduling");
        setBoolParam(param.DynamicPartitionScheduling, "DynamicPartitionScheduling");
        setFloatParam(param.robotReleaseBound, "robotReleaseBound");
        setBoolParam(param.FinalgameScheduling, "FinalgameScheduling");
        setIntParam(param.SHIP_WAIT_TIME_LIMIT, "SHIP_WAIT_TIME_LIMIT");
        setIntParam(param.GOOD_DISTANCE_LIMIT, "GOOD_DISTANCE_LIMIT");
        setIntParam(param.EARLY_DELIVERT_FRAME_LIMIT, "EARLY_DELIVERT_FRAME_LIMIT");
        setIntParam(param.EARLY_DELIVERY_VALUE_LIMIT, "EARLY_DELIVERY_VALUE_LIMIT");
    }

    void logParams(const Params &param){
        LOGI("设置的超参数：");
        LOGI(param.FINAL_FRAME, "FINAL_FRAME");
        LOGI(param.maxRobotNum, "maxRobotNum");
        LOGI(param.landDistanceWeight, "landDistanceWeight");
        LOGI(param.deliveryDistanceWeight, "deliveryDistanceWeight");
        LOGI(param.CentralizedTransportation, "CentralizedTransportation");
        LOGI(param.robotFirst, "robotFirst");
        LOGI(param.PartitionScheduling, "PartitionScheduling");
        LOGI(param.DynamicPartitionScheduling, "DynamicPartitionScheduling");
        LOGI(param.robotReleaseBound, "robotReleaseBound");
        LOGI(param.FinalgameScheduling, "FinalgameScheduling");
        LOGI(param.SHIP_WAIT_TIME_LIMIT, "SHIP_WAIT_TIME_LIMIT");
        LOGI(param.GOOD_DISTANCE_LIMIT, "GOOD_DISTANCE_LIMIT");
        LOGI(param.EARLY_DELIVERT_FRAME_LIMIT, "EARLY_DELIVERT_FRAME_LIMIT");
        LOGI(param.EARLY_DELIVERY_VALUE_LIMIT, "EARLY_DELIVERY_VALUE_LIMIT");
    }

    const std::unordered_map<std::string, std::string>& getParams() const {
        return params_;
    }

private:
    std::string filename_;
    std::unordered_map<std::string, std::string> params_;
};