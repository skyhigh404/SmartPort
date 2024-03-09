#pragma once
#include <string>
#include <fstream>
#include <chrono>
#include <iostream>
#include <cstring>
#include "assert.h"

// 定义日志级别
#define LOG_LEVEL_INFO 0
#define LOG_LEVEL_WARNING 1
#define LOG_LEVEL_ERROR 2
#define LOG_LEVEL_NONE 3

// 在这里设置当前的日志级别
#define LOG_LEVEL LOG_LEVEL_INFO

// 定义记录宏
#define LOGI(message) if (LOG_LEVEL <= LOG_LEVEL_INFO) Log::logWrite(message, LogLevel::INFO)
#define LOGW(message) if (LOG_LEVEL <= LOG_LEVEL_WARNING) Log::logWrite(message, LogLevel::WARNING)
#define LOGE(message) if (LOG_LEVEL <= LOG_LEVEL_ERROR) Log::logWrite(message, LogLevel::ERROR)


enum class LogLevel
{
    INFO,
    WARNING,
    ERROR
};

class Log
{
private:
    std::ofstream m_OutputStream;
    LogLevel m_LogLevel = LogLevel::INFO;

    Log() {}
    Log(const Log &) = delete;
    Log &operator=(const Log &) = delete;

public:
    ~Log()
    {
        endLog();
    }
    static Log &getInstance()
    {
        static Log log;
        return log;
    }

    static void initLog(const std::string &filepath = "log.log")
    {
        getInstance().initLogImpl(filepath);
    }
    static void endLog()
    {
        getInstance().endLogImpl();
    }
    static void logWrite(const std::string &message, LogLevel level = LogLevel::INFO)
    {
        getInstance().logWriteImpl(message, level);
    }

private:
    void initLogImpl(const std::string &filepath = "log.log")
    {
        m_OutputStream.open(filepath, std::ios::app); // 追加模式
        assert(m_OutputStream);
        writeHeader();
    }

    void endLogImpl()
    {
        if (m_OutputStream.is_open())
        {
            m_OutputStream.close();
        }
    }

    void writeHeader()
    {
        auto begin = std::chrono::system_clock::now();
        std::time_t begin_time = std::chrono::system_clock::to_time_t(begin);
        m_OutputStream << "===================================================================\n";
        m_OutputStream << "Log started at " << std::ctime(&begin_time);
        m_OutputStream << "===================================================================\n";
        m_OutputStream.flush();
    }

    void logWriteImpl(const std::string &message, LogLevel level)
    {
        if (level < m_LogLevel) // 如果日志级别低于当前设置的级别，则不记录
            return;

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        std::string timeStr = std::ctime(&now_time);
        if (!timeStr.empty() && timeStr[timeStr.length() - 1] == '\n') {
            timeStr.erase(timeStr.length() - 1); // 去除换行符
        }

        m_OutputStream << "[" << timeStr << "] ";

        // 根据日志级别添加不同的前缀
        switch (level)
        {
        case LogLevel::INFO:
            m_OutputStream << "[INFO]    ";
            break;
        case LogLevel::WARNING:
            m_OutputStream << "[WARNING] ";
            break;
        case LogLevel::ERROR:
            m_OutputStream << "[ERROR]   ";
            break;
        }

        m_OutputStream << message << "\n";
        m_OutputStream.flush();
    }
};