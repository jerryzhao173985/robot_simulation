#pragma once

#include <iostream>
#include <string>

// Debug output control
namespace Debug {
    enum Level {
        NONE = 0,
        ERROR = 1,
        WARNING = 2,
        INFO = 3,
        VERBOSE = 4,
        ALL = 5
    };
    
    // Global debug level - can be changed at runtime
    extern Level currentLevel;
    
    // Set debug level
    inline void setLevel(Level level) {
        currentLevel = level;
    }
    
    // Check if a message should be printed
    inline bool shouldPrint(Level msgLevel) {
        return msgLevel <= currentLevel;
    }
    
    // Debug output functions
    inline void error(const std::string& msg) {
        if (shouldPrint(ERROR)) {
            std::cerr << "[ERROR] " << msg << std::endl;
        }
    }
    
    inline void warning(const std::string& msg) {
        if (shouldPrint(WARNING)) {
            std::cerr << "[WARNING] " << msg << std::endl;
        }
    }
    
    inline void info(const std::string& msg) {
        if (shouldPrint(INFO)) {
            std::cout << "[INFO] " << msg << std::endl;
        }
    }
    
    inline void verbose(const std::string& msg) {
        if (shouldPrint(VERBOSE)) {
            std::cout << "[DEBUG] " << msg << std::endl;
        }
    }
    
    inline void physics(const std::string& msg) {
        if (shouldPrint(ALL)) {
            std::cout << "[PHYSICS] " << msg << std::endl;
        }
    }
}

// Macros for conditional debug output
#define DEBUG_INFO(msg) Debug::info(msg)
#define DEBUG_VERBOSE(msg) Debug::verbose(msg)
#define DEBUG_PHYSICS(msg) Debug::physics(msg)
#define DEBUG_WARNING(msg) Debug::warning(msg)
#define DEBUG_ERROR(msg) Debug::error(msg)