#pragma once

#include "../concurrency/monitored_thread.hpp"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
namespace wust_vl {
namespace common {
    namespace utils {
        class Timer {
        public:
            using Callback = std::function<void(double)>;

            Timer() = default;
            ~Timer() {
                stop();
            }

            void start(double rate_hz, Callback callback) {
                stop();

                interval_ = std::chrono::microseconds(static_cast<int64_t>(1e6 / rate_hz));
                callback_ = std::move(callback);

                thread_ = wust_vl::common::concurrency::MonitoredThread::create(
                    "TimerThread",
                    [this](wust_vl::common::concurrency::MonitoredThread::Ptr self) {
                        using clock = std::chrono::steady_clock;

                        auto start_time = clock::now();
                        auto last_exec_time = start_time;
                        auto next_time = start_time + interval_;

                        constexpr int MAX_CATCHUP_FRAMES = 10;
                        const auto max_catchup = interval_ * MAX_CATCHUP_FRAMES;

                        while (self->isAlive()) {
                            std::this_thread::sleep_until(next_time);

                            auto exec_time = clock::now();
                            double dt_ms = std::chrono::duration<double, std::milli>(
                                               exec_time - last_exec_time
                            )
                                               .count();
                            last_exec_time = exec_time;

                            self->heartbeat();

                            if (callback_)
                                callback_(dt_ms);

                            auto ideal_next_time = next_time + interval_;

                            auto min_next_time = last_exec_time;
                            auto max_next_time = last_exec_time + max_catchup;

                            next_time = std::clamp(ideal_next_time, min_next_time, max_next_time);
                        }
                    }
                );

                wust_vl::common::concurrency::ThreadManager::instance().registerThread(thread_);

                running_ = true;
            }

            void stop() {
                running_ = false;
                if (thread_) {
                    thread_->stop();
                    wust_vl::common::concurrency::ThreadManager::instance().unregisterThread(
                        thread_->getName()
                    );
                }
            }

            bool isRunning() const {
                return running_;
            }

        private:
            std::chrono::microseconds interval_;
            Callback callback_;
            wust_vl::common::concurrency::MonitoredThread::Ptr thread_;
            bool running_ = false;
        };

        namespace time_utils {

            using Clock = std::chrono::steady_clock;
            using TimePoint = Clock::time_point;

            /// --------- 全局程序启动时间点 ---------
            inline const TimePoint& programStartTime() {
                static const TimePoint start = Clock::now();
                return start;
            }

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

            /// 从程序启动到现在的毫秒差
            inline double sinceProgramStartMs() {
                return elapsedMs(programStartTime());
            }

            /// 从程序启动到现在的秒差
            inline double sinceProgramStartSec() {
                return durationSec(programStartTime(), now());
            }

            /// 格式化输出
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

            /// 当前系统时间字符串
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

            /// 相对程序启动时间的格式化字符串
            inline std::string sinceProgramStartStr() {
                std::ostringstream oss;
                oss << "[+" << std::fixed << std::setprecision(3) << sinceProgramStartMs()
                    << " ms]";
                return oss.str();
            }

        } // namespace time_utils
    } // namespace utils
} // namespace common
} // namespace wust_vl