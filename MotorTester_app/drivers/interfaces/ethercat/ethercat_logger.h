#pragma once

#include <iostream>
#include <string>

namespace ethercat_driver {

enum class LogLevel {
    Debug,
    Info,
    Warn,
    Error
};

inline void log(LogLevel level, const std::string& message) {
    switch (level) {
        case LogLevel::Debug:
            std::cout << "[DEBUG] " << message << std::endl;
            break;
        case LogLevel::Info:
            std::cout << "[INFO] " << message << std::endl;
            break;
        case LogLevel::Warn:
            std::cout << "[WARN] " << message << std::endl;
            break;
        case LogLevel::Error:
            std::cerr << "[ERROR] " << message << std::endl;
            break;
    }
}

} // namespace ethercat_driver
