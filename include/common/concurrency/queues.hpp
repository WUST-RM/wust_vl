#pragma once
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <vector>
template<typename T>
concept HasFrameIDAndTimestamp = requires(T a) {
    {
        a.id
        } -> std::convertible_to<int>;
    {
        a.timestamp
        } -> std::convertible_to<std::chrono::steady_clock::time_point>;
};

template<HasFrameIDAndTimestamp T>
class OrderedQueue {
public:
    OrderedQueue(int max_wait_ms = 50, int max_lag_ms = 200):
        current_id_(0),
        max_wait_ms_(max_wait_ms),
        max_lag_ms_(max_lag_ms) {}

    // 入队
    void enqueue(const T& item) {
        auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lk(mutex_);

            // 丢弃旧帧或滞后过久的帧
            if (item.id < current_id_)
                return;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - item.timestamp).count()
                > max_lag_ms_)
            {
                return;
            }

            Entry entry { item, now };
            buffer_[item.id] = std::move(entry);
        }
        cond_var_.notify_all(); // 唤醒等待线程
    }

    // 非阻塞尝试出队
    bool try_dequeue(T& out_item) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto now = std::chrono::steady_clock::now();

        if (buffer_.empty())
            return false;
        auto it = buffer_.begin();

        // 缺帧等待超时跳帧逻辑
        if (it->first > current_id_ + 1) {
            auto wait_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second.enqueue_time)
                    .count();
            if (wait_ms >= max_wait_ms_) {
                current_id_ = it->first;
            } else {
                return false;
            }
        }

        out_item = std::move(it->second.frame);
        current_id_ = it->first;
        buffer_.erase(it);
        return true;
    }

    // 阻塞式出队（新增）
    bool dequeue_wait(T& out_item, bool& timeout_skip) {
        timeout_skip = false;
        std::unique_lock<std::mutex> lk(mutex_);

        while (alive_) {
            cond_var_.wait(lk, [&] { return !buffer_.empty() || !alive_; });
            if (!alive_)
                break;
            if (buffer_.empty())
                continue;

            auto now = std::chrono::steady_clock::now();
            auto it = buffer_.begin();

            // 超时跳帧处理
            if (it->first > current_id_ + 1) {
                auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now - it->second.enqueue_time
                )
                                  .count();
                if (age_ms < max_wait_ms_) {
                    continue; // 继续等缺失帧
                }
                timeout_skip = true;
                current_id_ = it->first;
            }

            // 正常出队
            out_item = std::move(it->second.frame);
            current_id_ = it->first;
            buffer_.erase(it);
            return true;
        }

        return false;
    }

    void stop() {
        alive_ = false;
        cond_var_.notify_all();
    }

    bool is_alive() const noexcept {
        return alive_;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return buffer_.size();
    }

private:
    struct Entry {
        T frame;
        std::chrono::steady_clock::time_point enqueue_time;
    };

    std::map<int, Entry> buffer_;
    int current_id_;
    int max_wait_ms_;
    int max_lag_ms_;
    mutable std::mutex mutex_;
    std::condition_variable cond_var_;
    std::atomic<bool> alive_ { true };
};

template<typename T>
class TimedQueue {
public:
    struct QueueItem {
        T data;
        std::chrono::steady_clock::time_point timestamp;
        QueueItem(const T& d, const std::chrono::steady_clock::time_point& ts):
            data(d),
            timestamp(ts) {}

        QueueItem(T&& d, const std::chrono::steady_clock::time_point& ts):
            data(std::move(d)),
            timestamp(ts) {}
    };

    explicit TimedQueue(double valid_duration): valid_duration_(valid_duration) {}

    // 添加新目标（自动清理 + 唤醒消费线程）
    void push(
        const T& obj,
        std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now()
    ) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            clear_stale_locked();
            queue_.push_back({ obj, timestamp });
        }
        cv_.notify_one();
    }
    void push(
        T&& obj,
        std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now()
    ) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            clear_stale_locked();
            queue_.push_back({ std::move(obj), timestamp });
        }
        cv_.notify_one();
    }

    // 非阻塞 pop（你原来的逻辑）
    bool pop_valid(T& out) {
        std::lock_guard<std::mutex> lk(mtx_);
        clear_stale_locked();
        if (queue_.empty())
            return false;
        out = std::move(queue_.front().data);
        queue_.erase(queue_.begin());
        return true;
    }

    // 阻塞式 pop，直到有数据或队列被 stop
    bool pop_wait(T& out) {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [&] { return !queue_.empty() || !alive_; });

        clear_stale_locked();
        if (!alive_ || queue_.empty())
            return false;

        out = std::move(queue_.front().data);
        queue_.erase(queue_.begin());
        return true;
    }

    // 停止队列，唤醒所有等待线程
    void stop() {
        alive_ = false;
        cv_.notify_all();
    }

    bool is_alive() const noexcept {
        return alive_;
    }

private:
    void clear_stale_locked() {
        auto now = std::chrono::steady_clock::now();
        auto it = std::remove_if(queue_.begin(), queue_.end(), [&](const QueueItem& item) {
            double dt = std::chrono::duration<double>(now - item.timestamp).count();
            return dt > valid_duration_;
        });
        queue_.erase(it, queue_.end());
    }

    std::vector<QueueItem> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> alive_ { true };
    double valid_duration_;
};

template<typename T>
concept ArithmeticLike = requires(T a, T b, double n) {
    {
        a + b
        } -> std::same_as<T>; // 必须支持加法
    {
        a / n
        } -> std::convertible_to<T>; // 能被浮点数除并返回可转换为 T
};

template<ArithmeticLike T>
class Averager {
public:
    explicit Averager(size_t max_size): max_size_(max_size) {}

    void add(const T& value) {
        if (values_.size() >= max_size_) {
            values_.pop_front();
        }
        values_.push_back(value);
    }

    bool empty() const {
        return values_.empty();
    }
    size_t size() const {
        return values_.size();
    }

    /// 返回平均值
    T average() const {
        if (values_.empty()) {
            if constexpr (std::is_arithmetic_v<T>)
                return T(0);
            else
                return T {};
        }
        T sum = values_.front();
        for (size_t i = 1; i < values_.size(); ++i) {
            sum = sum + values_[i];
        }
        return sum / values_.size();
    }

    /// 清空
    void clear() {
        values_.clear();
    }

private:
    size_t max_size_;
    std::deque<T> values_;
};