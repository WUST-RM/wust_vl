#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
class MonitoredThread: public std::enable_shared_from_this<MonitoredThread> {
public:
    enum class Status { Running, Stopped, Hung, Paused };

    using Task = std::function<void(std::shared_ptr<MonitoredThread>)>;

    static std::shared_ptr<MonitoredThread> create(const std::string& name, Task task) {
        auto mt = std::shared_ptr<MonitoredThread>(new MonitoredThread(name));
        mt->start(task);
        return mt;
    }

    ~MonitoredThread() {
        stop();
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            running_ = false;
            status_ = Status::Stopped;
        }
        cv_.notify_all();
        if (th_.joinable())
            th_.join();
        if (monitor_.joinable())
            monitor_.join();
    }

    void heartbeat() {
        std::lock_guard<std::mutex> lk(mtx_);
        last_heartbeat_ = std::chrono::steady_clock::now();
    }

    Status getStatus() const {
        return status_;
    }
    std::string getName() const {
        return name_;
    }
    bool isAlive() const {
        return running_;
    }

    void pause() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (status_ == Status::Running)
            status_ = Status::Paused;
    }

    void resume() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (status_ == Status::Paused) {
            status_ = Status::Running;
            cv_.notify_all();
        }
    }

    bool waitPoint() {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this]() { return status_ != Status::Paused || !running_; });
        return running_;
    }

private:
    explicit MonitoredThread(const std::string& name): name_(name) {}

    void start(Task task) {
        running_ = true;
        status_ = Status::Running;
        last_heartbeat_ = std::chrono::steady_clock::now();

        // 工作线程
        th_ = std::thread([this, task]() {
            if (task)
                task(shared_from_this());
            running_ = false;
        });

        // 监控线程
        monitor_ = std::thread([this]() {
            while (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                std::lock_guard<std::mutex> lk(mtx_);

                // 如果线程暂停，则不检查心跳，不更新 Hung 状态
                if (status_ == Status::Paused)
                    continue;

                auto now = std::chrono::steady_clock::now();
                if (now - last_heartbeat_ > std::chrono::seconds(2)) {
                    status_ = Status::Hung;
                } else {
                    status_ = Status::Running;
                }
            }
        });
    }

    std::string name_;
    std::thread th_;
    std::thread monitor_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::atomic<bool> running_ { false };
    std::atomic<Status> status_ { Status::Stopped };
};

// ---------------------- ThreadManager ----------------------
class ThreadManager {
public:
    static ThreadManager& instance() {
        static ThreadManager inst;
        return inst;
    }

    void registerThread(const std::shared_ptr<MonitoredThread>& t) {
        std::lock_guard<std::mutex> lk(mtx_);
        threads_[t->getName()] = t;
    }

    void unregisterThread(const std::string& name) {
        std::lock_guard<std::mutex> lk(mtx_);
        threads_.erase(name);
    }

    void printStatus() {
        std::lock_guard<std::mutex> lk(mtx_);
        for (auto it = threads_.begin(); it != threads_.end();) {
            if (it->second.expired()) {
                it = threads_.erase(it);
            } else {
                auto t = it->second.lock();
                std::string status;
                switch (t->getStatus()) {
                    case MonitoredThread::Status::Running:
                        status = "Running";
                        break;
                    case MonitoredThread::Status::Stopped:
                        status = "Stopped";
                        break;
                    case MonitoredThread::Status::Hung:
                        status = "Hung";
                        break;
                    case MonitoredThread::Status::Paused:
                        status = "Paused";
                        break;
                }
                std::cout << "Thread [" << t->getName() << "] status: " << status << "\n";
                ++it;
            }
        }
    }
    std::map<std::string, MonitoredThread::Status> getAllThreadStatuses() {
        std::lock_guard<std::mutex> lk(mtx_);
        std::map<std::string, MonitoredThread::Status> statuses;
        for (auto it = threads_.begin(); it != threads_.end();) {
            if (it->second.expired()) {
                it = threads_.erase(it);
            } else {
                auto t = it->second.lock();

                statuses[t->getName()] = t->getStatus();
                ++it;
            }
        }
        return statuses;
    }

private:
    ThreadManager() = default;
    std::mutex mtx_;
    std::map<std::string, std::weak_ptr<MonitoredThread>> threads_;
};
