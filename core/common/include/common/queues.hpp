#pragma once
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <vector>

template<typename T>
class OrderedQueue {
public:
    static_assert(
        std::is_same<decltype(T::id), int>::value
            && std::is_same<decltype(T::timestamp), std::chrono::steady_clock::time_point>::value,
        "T must have int id and timestamp of type std::chrono::steady_clock::time_point"
    );

    OrderedQueue(int max_wait_ms = 50, int max_lag_ms = 200):
        current_id_(0),
        max_wait_ms_(max_wait_ms),
        max_lag_ms_(max_lag_ms) {}

    // 入队
    void enqueue(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();

        // 丢弃过期或滞后太久的帧
        if (item.id < current_id_
            || std::chrono::duration_cast<std::chrono::milliseconds>(now - item.timestamp).count()
                > max_lag_ms_)
        {
            return;
        }

        Entry entry { item, now };

        if (item.id < current_id_) {
            old_buffer_[item.id] = entry;
        } else {
            buffer_[item.id] = entry;
        }

        cond_var_.notify_one();
    }

    // 非阻塞尝试出队
    bool try_dequeue(T& out_item) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();

        // 清理 old_buffer 中滞后过久的帧
        while (!old_buffer_.empty()) {
            auto it_old = old_buffer_.begin();
            auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              now - it_old->second.enqueue_time
            )
                              .count();
            if (age_ms > max_lag_ms_) {
                old_buffer_.erase(it_old);
            } else {
                out_item = it_old->second.frame;
                old_buffer_.erase(it_old);
                return true;
            }
        }

        if (buffer_.empty())
            return false;

        auto it = buffer_.begin();

        // 如果缺失 ID 超时，则跳帧
        if (it->first > current_id_ + 1) {
            auto wait_duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second.enqueue_time)
                    .count();
            if (wait_duration >= max_wait_ms_) {
                current_id_ = it->first;
            } else {
                return false; // 等待缺失帧
            }
        }

        out_item = it->second.frame;
        current_id_ = it->first;
        buffer_.erase(it);
        return true;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size() + old_buffer_.size();
    }

private:
    struct Entry {
        T frame;
        std::chrono::steady_clock::time_point enqueue_time;
    };

    std::map<int, Entry> buffer_; // 按 ID 排序的主缓冲
    std::map<int, Entry> old_buffer_; // 小于 current_id_ 的旧缓冲
    int current_id_;
    int max_wait_ms_; // 缺帧等待时间
    int max_lag_ms_; // 帧滞后丢弃阈值（ms）
    std::mutex mutex_;
    std::condition_variable cond_var_;
};

template<typename T>
class TimedQueue {
public:
    struct QueueItem {
        T data;
        std::chrono::steady_clock::time_point timestamp;
    };

    TimedQueue(double valid_duration): valid_duration_(valid_duration) {}

    // 添加新目标
    void push(
        const T& obj,
        std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now()
    ) {
        std::lock_guard<std::mutex> lk(mtx_);
        queue_.push_back({ obj, timestamp });
    }

    // 获取时间有效目标（受 query_interval_ 控制）
    std::vector<T> get_valid_targets() {
        auto now = std::chrono::steady_clock::now();
        std::vector<T> valid_targets;

        std::lock_guard<std::mutex> lk(mtx_);
        if (queue_.empty())
            return valid_targets;

        for (const auto& item: queue_) {
            double dt = std::chrono::duration<double>(now - item.timestamp).count();
            if (dt <= valid_duration_) {
                valid_targets.push_back(item.data);
            }
        }

        last_query_ = now;
        return valid_targets;
    }

    // 清除超过 valid_duration_ 的目标
    void clear_stale() {
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = std::remove_if(queue_.begin(), queue_.end(), [&](const QueueItem& item) {
            double dt = std::chrono::duration<double>(now - item.timestamp).count();
            return dt > valid_duration_;
        });
        queue_.erase(it, queue_.end());
    }

private:
    std::vector<QueueItem> queue_;
    std::mutex mtx_;
    std::chrono::steady_clock::time_point last_query_ = std::chrono::steady_clock::now();
    double valid_duration_; // 有效时间窗口（receive_dt_）
};