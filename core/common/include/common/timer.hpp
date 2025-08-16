#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <mutex>
#include <thread>

class Timer {
public:
    using Callback = std::function<void(double)>;

    Timer() = default;

    ~Timer() {
        stop();
    }

    void start(
        double rate_hz,
        Callback callback,
        std::chrono::microseconds spin_margin = std::chrono::microseconds(200)
    ) {
        stop(); // 若已在运行则先停止

        interval_ = std::chrono::microseconds(static_cast<int64_t>(1e6 / rate_hz));
        callback_ = std::move(callback);
        running_ = true;

        thread_ = std::thread([this, spin_margin]() {
            auto next_time = std::chrono::steady_clock::now() + interval_;
            auto last_time = std::chrono::steady_clock::now();

            while (true) {
                {
                    std::unique_lock<std::mutex> lock(mtx_);
                    if (cv_.wait_until(lock, next_time - spin_margin, [this]() {
                            return !running_;
                        })) {
                        break;
                    }
                }

                while (std::chrono::steady_clock::now() < next_time) {
                }

                auto now = std::chrono::steady_clock::now();
                double dt_ms = std::chrono::duration<double, std::milli>(now - last_time).count();
                last_time = now;

                if (callback_) {
                    callback_(dt_ms);
                }

                next_time += interval_;
            }
        });
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            running_ = false;
        }
        cv_.notify_one();
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    bool isRunning() const {
        return running_;
    }

private:
    std::chrono::microseconds interval_;
    Callback callback_;
    std::thread thread_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool running_ = false;
};
namespace time_utils {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

/// 获取当前时间点
inline TimePoint now() {
    return Clock::now();
}

/// 毫秒、微秒、秒间隔
inline double durationMs(const TimePoint& start, const TimePoint& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

inline double durationUs(const TimePoint& start, const TimePoint& end) {
    return std::chrono::duration<double, std::micro>(end - start).count();
}

inline double durationSec(const TimePoint& start, const TimePoint& end) {
    return std::chrono::duration<double>(end - start).count();
}

/// 从时间点到现在的毫秒数
inline double elapsedMs(const TimePoint& start) {
    return durationMs(start, now());
}

/// 格式化时间差为字符串（毫秒，3位精度）
inline std::string formatDurationMs(const TimePoint& start, const TimePoint& end) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << durationMs(start, end) << " ms";
    return oss.str();
}

inline std::string formatDurationUs(const TimePoint& start, const TimePoint& end) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << durationUs(start, end) << " us";
    return oss.str();
}

inline std::string formatDurationSec(const TimePoint& start, const TimePoint& end) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << durationSec(start, end) << " s";
    return oss.str();
}

/// 获取系统当前时间（wall clock）并格式化为字符串
inline std::string currentSystemTimeStr(const char* format = "%Y-%m-%d %H:%M:%S") {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#if defined(_MSC_VER)
    localtime_s(&tm_buf, &now_time_t);
#else
    localtime_r(&now_time_t, &tm_buf);
#endif
    char buf[64];
    std::strftime(buf, sizeof(buf), format, &tm_buf);
    return std::string(buf);
}

/// 获取当前时间点（steady_clock）偏移毫秒后的字符串（相对格式）
inline std::string offsetTimePointStr(const TimePoint& base, const char* label = "ΔT") {
    std::ostringstream oss;
    oss << label << ": +" << std::fixed << std::setprecision(3) << elapsedMs(base) << " ms";
    return oss.str();
}

} // namespace time_utils