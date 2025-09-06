// Copyright 2025 XiaoJian Wu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Copyright 2025 XiaoJian Wu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...

#pragma once

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
namespace fs = std::filesystem;

// ========== 日志等级 ==========
enum class LogLevel { DEBUG = 0, INFO, WARN, ERROR, MAIN };

inline const char* levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::MAIN:
            return "MAIN";
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return " INFO";
        case LogLevel::WARN:
            return " WARN";
        case LogLevel::ERROR:
            return "ERROR";
        default:
            return "UNKN ";
    }
}

inline const char* colorForLevel(LogLevel level) {
    switch (level) {
        case LogLevel::MAIN:
            return "\033[37m"; // 白色
        case LogLevel::DEBUG:
            return "\033[36m"; // 青色
        case LogLevel::INFO:
            return "\033[32m"; // 绿色
        case LogLevel::WARN:
            return "\033[33m"; // 黄色
        case LogLevel::ERROR:
            return "\033[31m"; // 红色
        default:
            return "\033[0m";
    }
}

inline const char* colorReset() {
    return "\033[0m";
}

inline LogLevel logLevelFromString(const std::string& level_str) {
    std::string l = level_str;
    std::transform(l.begin(), l.end(), l.begin(), ::toupper);
    if (l == "MAIN")
        return LogLevel::MAIN;
    if (l == "DEBUG")
        return LogLevel::DEBUG;
    if (l == "INFO")
        return LogLevel::INFO;
    if (l == "WARN")
        return LogLevel::WARN;
    if (l == "ERROR")
        return LogLevel::ERROR;
    throw std::invalid_argument("Invalid log level string: " + level_str);
}

// 时间戳
inline std::string getTimeStr() {
    auto now = std::chrono::system_clock::now();
    auto now_tt = std::chrono::system_clock::to_time_t(now);
    auto now_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_tt), "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0')
        << std::setw(3) << now_ms.count();
    return oss.str();
}

// ========== 核心 Logger ==========
class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLevel(const std::string& level_str) {
        setLevel(logLevelFromString(level_str));
    }
    void setLevel(LogLevel level) {
        log_level_ = level;
    }

    void enableFileOutput(const std::string& filename) {
        std::lock_guard<std::mutex> lock(mutex_);
        file_stream_.open(filename, std::ios::out | std::ios::app);
        file_output_enabled_ = file_stream_.is_open();
    }

    void disableColorOutput() {
        color_output_enabled_ = false;
    }
    void enableSimplifiedOutput(bool enabled) {
        simplified_output_enabled_ = enabled;
    }

    LogLevel getLevel() const {
        return log_level_;
    }
    bool shouldLog(LogLevel level) const {
        return level >= log_level_;
    }
    bool isSimplifiedOutputEnabled() const {
        return simplified_output_enabled_;
    }
    std::ofstream& fileStream() {
        return file_stream_;
    }
    bool isFileOutputEnabled() const {
        return file_output_enabled_;
    }
    bool isColorOutputEnabled() const {
        return color_output_enabled_;
    }
    std::mutex& getMutex() {
        return mutex_;
    }

private:
    Logger():
        log_level_(LogLevel::DEBUG),
        file_output_enabled_(false),
        color_output_enabled_(true),
        simplified_output_enabled_(false) {}
    ~Logger() {
        if (file_stream_.is_open())
            file_stream_.close();
    }

    LogLevel log_level_;
    bool file_output_enabled_;
    bool color_output_enabled_;
    bool simplified_output_enabled_;
    std::ofstream file_stream_;
    mutable std::mutex mutex_;
};

// ========== 流式日志类 ==========
class LoggerStream {
public:
    LoggerStream(LogLevel level, const std::string& node, const char* file, int line):
        level_(level),
        node_name_(node),
        file_(file),
        line_(line) {}

    ~LoggerStream() {
        // 如果是流式抛异常，析构不输出
        if (throwing_)
            return;

        std::ostringstream full_msg;
        if (level_ == LogLevel::MAIN || !Logger::getInstance().isSimplifiedOutputEnabled()) {
            full_msg << "[" << getTimeStr() << "]"
                     << "[" << levelToString(level_) << "]"
                     << "[" << node_name_ << "]"
                     << "[" << file_ << ":" << line_ << "] " << buffer_.str();
        } else {
            full_msg << "[" << levelToString(level_) << "]"
                     << "[" << node_name_ << "] " << buffer_.str();
        }

        std::lock_guard<std::mutex> lock(Logger::getInstance().getMutex());

        if (Logger::getInstance().shouldLog(level_)) {
            if (Logger::getInstance().isColorOutputEnabled()) {
                std::cout << colorForLevel(level_) << full_msg.str() << colorReset() << std::endl;
            } else {
                std::cout << full_msg.str() << std::endl;
            }
        }

        if (Logger::getInstance().isFileOutputEnabled()) {
            Logger::getInstance().fileStream() << full_msg.str() << std::endl;
        }
    }

    template<typename T>
    LoggerStream& operator<<(const T& val) {
        buffer_ << val;
        return *this;
    }

    // 新增流式抛异常方法
    [[noreturn]] void throwError() {
        throwing_ = true;
        std::ostringstream full_msg;
        full_msg << buffer_.str();
        logAndThrow(full_msg.str(), node_name_, file_, line_);
    }
    inline void
    logAndThrow(const std::string& msg, const std::string& node, const char* file, int line) {
        std::ostringstream oss;
        oss << msg;

        std::lock_guard<std::mutex> lock(Logger::getInstance().getMutex());
        std::ostringstream full_msg;
        full_msg << "[" << getTimeStr() << "]"
                 << "[ERROR]"
                 << "[" << node << "]"
                 << "[" << file << ":" << line << "] " << oss.str();

        if (Logger::getInstance().isColorOutputEnabled()) {
            std::cerr << colorForLevel(LogLevel::ERROR) << full_msg.str() << colorReset()
                      << std::endl;
        } else {
            std::cerr << full_msg.str() << std::endl;
        }

        if (Logger::getInstance().isFileOutputEnabled()) {
            Logger::getInstance().fileStream() << full_msg.str() << std::endl;
        }

        throw std::runtime_error(oss.str());
    }

private:
    LogLevel level_;
    std::ostringstream buffer_;
    std::string node_name_;
    const char* file_;
    int line_;
    bool throwing_ = false;
};

inline void initLogger(
    const std::string& level_str,
    const std::string& log_dir = "./logs",
    bool use_logcli = true,
    bool use_logfile = true,
    bool simplified_output = false
) {
    Logger& logger = Logger::getInstance();
    logger.setLevel(logLevelFromString(level_str));
    logger.enableSimplifiedOutput(simplified_output);

    if (!use_logcli)
        logger.setLevel(LogLevel::ERROR);

    fs::path dir_path(log_dir);
    if (!dir_path.is_absolute())
        dir_path = fs::absolute(dir_path);

    std::string timestamp = getTimeStr();
    std::replace(timestamp.begin(), timestamp.end(), ':', '-');
    std::replace(timestamp.begin(), timestamp.end(), ' ', '_');
    fs::path filename = dir_path / ("log_" + timestamp + ".txt");
    fs::create_directories(dir_path);

    if (use_logfile)
        logger.enableFileOutput(filename.string());
}

// ========== 宏定义 ==========
#define WUST_MAIN(node) LoggerStream(LogLevel::MAIN, node, __FILE__, __LINE__)
#define WUST_DEBUG(node) LoggerStream(LogLevel::DEBUG, node, __FILE__, __LINE__)
#define WUST_INFO(node) LoggerStream(LogLevel::INFO, node, __FILE__, __LINE__)
#define WUST_WARN(node) LoggerStream(LogLevel::WARN, node, __FILE__, __LINE__)
#define WUST_ERROR(node) LoggerStream(LogLevel::ERROR, node, __FILE__, __LINE__)
#define WUST_THROW_ERROR_STREAM(node) \
    LoggerStream(LogLevel::ERROR, node, __FILE__, __LINE__).throwError()
