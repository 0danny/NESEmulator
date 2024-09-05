#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>

extern std::mutex mutex;

namespace Utils {

    class Logger {
    public:
        enum class LogLevel {
            DEBUG,
            INFO,
            WARNING,
            ERROR
        };

        // Variadic template function for logging
        template<typename... Args>
        static void Debug(const std::string& message, Args... args) {
            Log(LogLevel::DEBUG, Format(message, args...));
        }

        template<typename... Args>
        static void Info(const std::string& message, Args... args) {
            Log(LogLevel::INFO, Format(message, args...));
        }

        template<typename... Args>
        static void Warning(const std::string& message, Args... args) {
            Log(LogLevel::WARNING, Format(message, args...));
        }

        template<typename... Args>
        static void Error(const std::string& message, Args... args) {
            Log(LogLevel::ERROR, Format(message, args...));
        }

        static std::string Uint8ToHex(uint8_t value) {
            std::stringstream ss;
            ss << "0x"
                << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value);
            return ss.str();
        }

        static std::string Uint16ToHex(uint16_t value) {
            std::stringstream ss;
            ss << "0x"
                << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value);
            return ss.str();
        }

        static std::string Uint8ToHexUppercase(uint8_t value) {
            std::stringstream ss;
            ss << std::uppercase  // Make the output uppercase
                << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value);
            return ss.str();
        }

        static std::string Uint16ToHexUppercase(uint16_t value) {
            std::stringstream ss;
            ss << std::uppercase  // Make the output uppercase
                << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(value);
            return ss.str();
        }

    private:
        static void Log(LogLevel level, const std::string& message) {
            std::lock_guard<std::mutex> lock(mutex);
            std::string logMessage = "[" + GetTimestamp() + "][" +
                GetLogLevel(level) + "] " +
                message + "\n";
            std::cout << logMessage;
            std::cout.flush();
        }

        static std::string GetTimestamp() {
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&now_c), "%H:%M:%S");
            return ss.str();
        }

        static std::string GetLogLevel(LogLevel level) {
            switch (level) {
            case LogLevel::DEBUG:   return "DEBUG";
            case LogLevel::INFO:    return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR:   return "ERROR";
            default:                return "UNKNOWN";
            }
        }

        // Variadic template function to format the message
        template<typename... Args>
        static std::string Format(const std::string& format, Args... args) {
            std::stringstream ss;
            ((ss << args), ...);
            return format + ss.str();
        }
    };
}
