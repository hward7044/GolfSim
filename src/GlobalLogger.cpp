#include "GlobalLogger.hpp"
GlobalLogger* GlobalLogger::instance = nullptr;
GlobalLogger* GlobalLogger::getInstance() {
    if (!instance) { instance = new GlobalLogger(); }
    return instance;
}
void GlobalLogger::logInfo(const std::string& msg) {}
void GlobalLogger::logError(const std::string& msg) {}
void GlobalLogger::setLevel(LogLevel level) {}
