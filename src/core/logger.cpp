#include "logger.h"

Logger &Logger::instance()
{
    static Logger instance;
    return instance;
}

void Logger::log(LogLevel level, const std::string &message)
{
    std::string level_str;
    switch (level)
    {
    case LogLevel::Info:
        level_str = "[INFO] ";
        break;
    case LogLevel::Warn:
        level_str = "[WARN] ";
        break;
    case LogLevel::Error:
        level_str = "[ERROR] ";
        break;
    }

    std::string full_message = level_str + message;
    std::cout << full_message << std::endl;

    for (auto &cb : callbacks_)
    {
        cb({level, full_message});
    }
}

void Logger::registerCallback(LogCallback cb)
{
    callbacks_.push_back(cb);
}