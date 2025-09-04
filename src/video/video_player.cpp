// Copyright 2025 Xiaojian Wu
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
#include "video/video_player.hpp"
#include "common/utils/logger.hpp"
#include <chrono>
#include <iostream>
#include <thread>
namespace wust_vl_video {
VideoPlayer::VideoPlayer(const std::string& video_path, int frame_rate, int start_frame, bool loop):
    path_(video_path),
    frame_rate_(frame_rate),
    start_frame_(start_frame),
    loop_(loop),
    running_(false),
    trigger_mode_(false) {}

void VideoPlayer::setCallback(FrameCallback cb) {
    on_frame_callback_ = std::move(cb);
}
void VideoPlayer::enableTriggerMode(bool enable) {
    trigger_mode_ = enable;
}

bool VideoPlayer::start() {
    cap_.open(path_);
    if (!cap_.isOpened()) {
        WUST_ERROR("video") << "Failed to open video: " << path_;
        return false;
    }
    WUST_INFO("video") << "Video opened successfully: " << path_;

    cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
    running_ = true;

    if (!trigger_mode_) {
        worker_ = std::thread(&VideoPlayer::run, this);
    }

    return true;
}
bool VideoPlayer::read() {
    if (!trigger_mode_ || !cap_.isOpened()) {
        return false;
    }

    cv::Mat frame_bgr;
    cap_ >> frame_bgr;

    if (frame_bgr.empty()) {
        if (loop_) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
            cap_ >> frame_bgr;
            if (frame_bgr.empty())
                return false;
        } else {
            return false;
        }
    }
    //cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_BGR2RGB);
    ImageFrame frame;
    frame.src_img = std::move(frame_bgr);
    frame.timestamp = std::chrono::steady_clock::now();

    if (on_frame_callback_) {
        on_frame_callback_(frame);
    }

    return true;
}

void VideoPlayer::stop() {
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
    cap_.release();
}

VideoPlayer::~VideoPlayer() {
    stop();
}

void VideoPlayer::run() {
    using clock = std::chrono::steady_clock;

    const auto frame_interval =
        std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(1.0 / frame_rate_)
        );

    auto next_frame_time = clock::now();

    while (running_) {
        cv::Mat frame_bgr;
        cap_ >> frame_bgr;

        if (frame_bgr.empty()) {
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
                continue;
            } else {
                break;
            }
        }

        //cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_BGR2RGB);

        ImageFrame frame;
        frame.src_img = std::move(frame_bgr);
        frame.timestamp = clock::now();

        if (on_frame_callback_) {
            on_frame_callback_(frame);
        }

        next_frame_time += frame_interval;
        std::this_thread::sleep_until(next_frame_time);
    }
}
} // namespace wust_vl_video