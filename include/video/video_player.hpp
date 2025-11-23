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
#pragma once
#include "video/image.hpp"
#include <atomic>
#include <functional>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
namespace wust_vl_video {
class VideoPlayer {
public:
    using FrameCallback = std::function<void(ImageFrame&)>;

    VideoPlayer(
        const std::string& video_path,
        int frame_rate = 30,
        int start_frame = 0,
        bool loop = true
    );

    void setCallback(FrameCallback cb);
    bool start();
    void stop();
    bool read();
    void enableTriggerMode(bool enable);
    void setCvtFlag(bool use_cvt)
    {
        use_cvt_ = use_cvt;

    }
    ~VideoPlayer();

private:
    void run(); // 后台线程函数

    std::string path_;
    int frame_rate_;
    int start_frame_;
    bool loop_;
    std::atomic<bool> running_;
    cv::VideoCapture cap_;
    std::thread worker_;
    bool trigger_mode_ = false;
    FrameCallback on_frame_callback_;
    bool use_cvt_= false;
};
} // namespace wust_vl_video