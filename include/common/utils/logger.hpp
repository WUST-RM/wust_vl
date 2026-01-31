#pragma once

#ifdef DEBUG
    #undef DEBUG
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace fs = std::filesystem;

enum class LogLevel : uint8_t { DEBUG = 0, INFO, WARN, ERROR, MAIN };

constexpr std::array<std::string_view, 5> kLevelNames { "DEBUG",
                                                        " INFO",
                                                        " WARN",
                                                        "ERROR",
                                                        "MAIN" };

constexpr std::array<std::string_view, 5> kLevelColors {
    "\033[36m", // DEBUG
    "\033[32m", // INFO
    "\033[33m", // WARN
    "\033[31m", // ERROR
    "\033[37m" // MAIN
};

constexpr std::string_view kColorReset = "\033[0m";

inline constexpr std::string_view levelToString(LogLevel level) noexcept {
    return kLevelNames[static_cast<size_t>(level)];
}

inline constexpr std::string_view colorForLevel(LogLevel level) noexcept {
    return kLevelColors[static_cast<size_t>(level)];
}

inline std::string getTimeStr() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const auto tt = clock::to_time_t(now);
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm {};
    localtime_r(&tt, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0')
        << ms.count();
    return oss.str();
}

class Logger {
public:
    static Logger& getInstance() {
        static Logger inst;
        return inst;
    }

    void setLevel(LogLevel level) noexcept {
        log_level_ = level;
    }

    void setLevel(const std::string& level_str) {
        std::string s = level_str;
        std::transform(s.begin(), s.end(), s.begin(), ::toupper);

        if (s == "DEBUG")
            setLevel(LogLevel::DEBUG);
        else if (s == "INFO")
            setLevel(LogLevel::INFO);
        else if (s == "WARN")
            setLevel(LogLevel::WARN);
        else if (s == "ERROR")
            setLevel(LogLevel::ERROR);
        else if (s == "MAIN")
            setLevel(LogLevel::MAIN);
        else
            throw std::invalid_argument("Invalid log level: " + level_str);
    }

    void enableFileOutput(const std::string& filename) {
        std::scoped_lock lock(mutex_);
        file_.open(filename, std::ios::app);
        file_enabled_ = file_.is_open();
    }

    void disableColorOutput() noexcept {
        color_enabled_ = false;
    }
    void enableSimplifiedOutput(bool v) noexcept {
        simplified_ = v;
    }

    bool shouldLog(LogLevel l) const noexcept {
        return l >= log_level_;
    }
    bool fileEnabled() const noexcept {
        return file_enabled_;
    }
    bool colorEnabled() const noexcept {
        return color_enabled_;
    }
    bool simplified() const noexcept {
        return simplified_;
    }

    std::ofstream& file() noexcept {
        return file_;
    }
    std::mutex& mutex() noexcept {
        return mutex_;
    }

private:
    Logger() = default;
    ~Logger() {
        if (file_.is_open())
            file_.close();
    }

    LogLevel log_level_ { LogLevel::DEBUG };
    bool file_enabled_ { false };
    bool color_enabled_ { true };
    bool simplified_ { false };

    std::ofstream file_;
    std::mutex mutex_;
};

class LoggerStream {
public:
    LoggerStream(LogLevel lvl, std::string node, const char* file, int line):
        level_(lvl),
        node_(std::move(node)),
        file_(file),
        line_(line) {}

    ~LoggerStream() {
        if (throwing_) {
            flushAndThrow();
            return;
        }
        flush();
    }

    template<typename T>
    LoggerStream& operator<<(const T& v) {
        buf_ << v;
        return *this;
    }

    LoggerStream& throwError() noexcept {
        throwing_ = true;
        return *this;
    }

private:
    void flush() {
        Logger& log = Logger::getInstance();
        if (!log.shouldLog(level_))
            return;

        std::ostringstream out;
        if (level_ == LogLevel::MAIN || !log.simplified()) {
            out << "[" << getTimeStr() << "]"
                << "[" << levelToString(level_) << "]"
                << "[" << node_ << "]"
                << "[" << file_ << ":" << line_ << "] " << buf_.str();
        } else {
            out << "[" << levelToString(level_) << "]"
                << "[" << node_ << "] " << buf_.str();
        }

        std::scoped_lock lock(log.mutex());

        if (log.colorEnabled()) {
            std::cout << colorForLevel(level_) << out.str() << kColorReset << std::endl;
        } else {
            std::cout << out.str() << std::endl;
        }

        if (log.fileEnabled()) {
            log.file() << out.str() << std::endl;
        }
    }

    [[noreturn]] void flushAndThrow() {
        flush();
        throw std::runtime_error(buf_.str());
    }

    LogLevel level_;
    std::ostringstream buf_;
    std::string node_;
    const char* file_;
    int line_;
    bool throwing_ { false };
};

inline void initLogger(
    const std::string& level,
    const std::string& dir = "./logs",
    bool cli = true,
    bool file = true,
    bool simplified = false
) {
    auto& log = Logger::getInstance();
    log.setLevel(level);
    log.enableSimplifiedOutput(simplified);

    if (!cli)
        log.setLevel(LogLevel::ERROR);

    fs::create_directories(dir);
    if (!file)
        return;

    auto name = getTimeStr();
    std::replace(name.begin(), name.end(), ':', '-');
    std::replace(name.begin(), name.end(), ' ', '_');

    log.enableFileOutput((fs::path(dir) / ("log_" + name + ".txt")).string());
}

#define WUST_MAIN(node) LoggerStream(LogLevel::MAIN, node, __FILE__, __LINE__)
#define WUST_DEBUG(node) LoggerStream(LogLevel::DEBUG, node, __FILE__, __LINE__)
#define WUST_INFO(node) LoggerStream(LogLevel::INFO, node, __FILE__, __LINE__)
#define WUST_WARN(node) LoggerStream(LogLevel::WARN, node, __FILE__, __LINE__)
#define WUST_ERROR(node) LoggerStream(LogLevel::ERROR, node, __FILE__, __LINE__)
#define WUST_THROW_ERROR(node) LoggerStream(LogLevel::ERROR, node, __FILE__, __LINE__).throwError()