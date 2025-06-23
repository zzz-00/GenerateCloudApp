#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>

enum class LogLevel
{
    Info,
    Warn,
    Error
};

struct LogMessage
{
    LogLevel level;
    std::string message;
};

class Logger
{
public:
    using LogCallback = std::function<void(const LogMessage &)>;

    static Logger &instance();
    void log(LogLevel level, const std::string &message);
    void registerCallback(LogCallback cb);

private:
    Logger() = default;
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    std::vector<LogCallback> callbacks_;
};
