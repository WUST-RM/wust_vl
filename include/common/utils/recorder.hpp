#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

#if defined(_WIN32)
    #include <io.h>
    #include <sys/stat.h>
#else
    #include <sys/stat.h>
    #include <unistd.h>
#endif

namespace wust_vl {
namespace common {
    namespace utils {

        template<typename T>
        class Writer {
        public:
            virtual ~Writer() = default;
            virtual void write(std::ostream& os, const T& data) = 0;
        };

        template<typename T>
        class Parser {
        public:
            virtual ~Parser() = default;
            virtual bool read(std::istream& is, T& out) = 0;
        };

        template<typename T>
        class Recorder {
        public:
            using WriterPtr = std::shared_ptr<Writer<T>>;

            Recorder(const std::filesystem::path& path, WriterPtr writer):
                path_(path),
                writer_(std::move(writer)),
                running_(false) {}

            ~Recorder() {
                stop();
                setFilePermission(path_);
            }

            void start() {
                if (running_)
                    return;
                if (!path_.empty()) {
                    if (!std::filesystem::exists(path_.parent_path())) {
                        std::filesystem::create_directories(path_.parent_path());
                    }

                    stream_.open(path_, std::ios::out | std::ios::binary | std::ios::app);
                    if (!stream_.is_open()) {
                        throw std::runtime_error("Cannot open recorder file: " + path_.string());
                    }

                    setFilePermission(path_);
                }

                start_time_ = std::chrono::steady_clock::now();
                running_ = true;
                worker_ = std::thread(&Recorder::workerLoop, this);
            }

            void stop() {
                if (!running_)
                    return;
                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    running_ = false;
                }
                cv_.notify_all();
                if (worker_.joinable())
                    worker_.join();
                stream_.close();
            }

            void push(const T& data) {
                {
                    std::lock_guard<std::mutex> lk(mtx_);
                    if (queue_.size() < 200)
                        queue_.emplace_back(data);
                }
                cv_.notify_one();
            }

            bool isRunning() const noexcept {
                return running_;
            }

        private:
            void workerLoop() {
                while (true) {
                    T data;
                    {
                        std::unique_lock<std::mutex> lk(mtx_);
                        cv_.wait(lk, [this] { return !queue_.empty() || !running_; });
                        if (!running_ && queue_.empty())
                            break;
                        data = std::move(queue_.front());
                        queue_.pop_front();
                    }
                    writer_->write(stream_, data);
                    stream_.flush();
                }
            }

            static void setFilePermission(const std::filesystem::path& p) {
#if defined(_WIN32)
                if (_chmod(p.string().c_str(), _S_IREAD | _S_IWRITE) != 0) {
                    std::cerr << "[Recorder] Warning: failed to set file permission for " << p
                              << std::endl;
                }
#else
                if (::chmod(p.c_str(), 0777) != 0) { // rw-rw-rw-
                    std::cerr << "[Recorder] Warning: failed to chmod " << p << std::endl;
                }
#endif
            }

        private:
            std::filesystem::path path_;
            std::ofstream stream_;
            WriterPtr writer_;
            std::deque<T> queue_;
            std::mutex mtx_;
            std::condition_variable cv_;
            std::thread worker_;
            std::atomic<bool> running_;
            std::chrono::steady_clock::time_point start_time_;
        };

        template<typename T>
        class RecorderParser {
        public:
            using ParserPtr = std::shared_ptr<Parser<T>>;

            explicit RecorderParser(const std::filesystem::path& path, ParserPtr parser):
                path_(path),
                parser_(std::move(parser)) {}

            bool open() {
                stream_.open(path_, std::ios::in | std::ios::binary);
                if (!stream_.is_open()) {
                    throw std::runtime_error("Cannot open recorder file: " + path_.string());
                }
                return true;
            }

            bool readNext(T& data) {
                if (!parser_)
                    return false;
                if (!stream_.good())
                    return false;
                return parser_->read(stream_, data);
            }

            void close() {
                if (stream_.is_open())
                    stream_.close();
            }

        private:
            std::filesystem::path path_;
            std::ifstream stream_;
            ParserPtr parser_;
        };

    } // namespace utils
} // namespace common
} // namespace wust_vl