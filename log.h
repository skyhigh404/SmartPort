#pragma once
#include <string>
#include <fstream>
#include <chrono>
#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <mutex>
#include "assert.h"

// 定义日志级别
#define LOG_LEVEL_INFO 0
#define LOG_LEVEL_WARNING 1
#define LOG_LEVEL_ERROR 2
#define LOG_LEVEL_NONE 3

// 在这里设置当前的日志级别
#define LOG_LEVEL LOG_LEVEL_INFO

#ifdef DEBUG
// 定义记录宏
#define LOGI(...) Log::logWriteFormatted(LogLevel::INFO, __VA_ARGS__)
#define LOGW(...) Log::logWriteFormatted(LogLevel::WARNING, __VA_ARGS__)
#define LOGE(...) Log::logWriteFormatted(LogLevel::ERROR, __VA_ARGS__)

#else
// 如果没有定义DEBUG，则将LOG宏定义为不执行任何操作的宏
// 使用do { } while(0)是为了确保宏在使用时的语义正确性，比如在if语句中不会出现语法错误
#define LOGI(...) \
    do            \
    {             \
    } while (0)
#define LOGW(...) \
    do            \
    {             \
    } while (0)
#define LOGE(...) \
    do            \
    {             \
    } while (0)

#endif

enum class LogLevel
{
    INFO,
    WARNING,
    ERROR
};

extern int CURRENT_FRAME;

class Log
{
private:
    std::ofstream m_OutputStream;
    LogLevel m_LogLevel = LogLevel::INFO;
    std::mutex writeMutex;

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
        auto &instance = getInstance();
        std::lock_guard<std::mutex> lock(instance.writeMutex);
        instance.logWriteImpl(message, level);
    }

    // 一个辅助函数，用于递归终止
    static void logMessage(std::ostringstream &stream)
    {
        // 递归终止时不做任何事
    }

    // 递归地将参数添加到输出流中
    template <typename T, typename... Args>
    static void logMessage(std::ostringstream &stream, const T &value, const Args &...args)
    {
        stream << value;
        logMessage(stream, args...);
    }

    template <typename... Args>
    static void logWriteFormatted(LogLevel level, const Args &...args)
    {
        std::ostringstream stream;
        logMessage(stream, args...);
        logWrite(stream.str(), level);
    }

    template <typename T>
    static std::string printVector(const std::vector<T> &vec)
    {
        std::ostringstream oss;
        for (const auto &val : vec)
            oss << val << " ";
        return oss.str();
    }

private:
    void initLogImpl(const std::string &filepath = "log.log")
    {
        m_OutputStream.open(filepath, std::ios::out); // 追加模式
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

        // auto now = std::chrono::system_clock::now();
        // std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        // std::string timeStr = std::ctime(&now_time);
        // if (!timeStr.empty() && timeStr[timeStr.length() - 1] == '\n')
        // {
        //     timeStr.erase(timeStr.length() - 1); // 去除换行符
        // }

        // m_OutputStream << "[" << timeStr << "] ";

        m_OutputStream << "[" << std::setw(5) << CURRENT_FRAME << "] ";

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