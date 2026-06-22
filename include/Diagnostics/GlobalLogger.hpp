#pragma once
#include <spdlog/spdlog.h>
#include <memory>
#include <string>
#include "Diagnostics/LogLevel.hpp"
namespace spdlog { namespace details { class thread_pool; } }
class GlobalLogger {
private:
    std::shared_ptr<spdlog::details::thread_pool> asyncQueue;
    static GlobalLogger* instance;
    GlobalLogger() = default;
public:
    static GlobalLogger* getInstance();
    void logInfo(const std::string& msg);
    void logError(const std::string& msg);
    void setLevel(LogLevel level);
};
