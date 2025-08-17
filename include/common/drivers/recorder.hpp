// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
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

#pragma once
// std
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>
// OpenCV
#include <opencv2/videoio.hpp>

class Recorder {
public:
    using imgFrame = std::vector<unsigned char>;
    Recorder(const std::filesystem::path& file, int fps, cv::Size size);
    ~Recorder();

    void addFrame(const imgFrame& frame);
    void start();
    void stop();

    std::filesystem::path path;

private:
    void recorderThread();

    cv::Size size_;
    int fps_;
    cv::VideoWriter writer_;

    std::deque<imgFrame> frame_queue_;

    std::mutex mutex_;
    std::atomic<bool> recoring_;
    std::condition_variable cv_;
    std::thread recorder_thread_;
};
