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
#include <chrono>
#include <iostream>
#include <thread>

namespace wust_vl {

namespace video {
    VideoPlayer::VideoPlayer() {}
    bool VideoPlayer::loadConfig(const YAML::Node& config) {
        path_ = config["path"].as<std::string>("");
        frame_rate_ = config["fps"].as<int>(30);
        start_frame_ = config["start_frame"].as<int>(0);
        loop_ = config["loop"].as<bool>(false);
        use_cvt_ = config["use_cvt"].as<bool>(false);
        trigger_mode_ = config["trigger_mode"].as<bool>(false);
        return true;
    }
    void VideoPlayer::setFrameCallback(FrameCallback cb) {
        on_frame_callback_ = std::move(cb);
    }

    void VideoPlayer::start() {
        cap_.open(path_);
        if (!cap_.isOpened()) {
            WUST_ERROR("video") << "Failed to open video: " << path_;
            return;
        }
        WUST_INFO("video") << "Video opened successfully: " << path_;

        cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
        running_ = true;

        if (!trigger_mode_) {
            worker_ = std::thread(&VideoPlayer::run, this);
        }
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
        if (use_cvt_)
            cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_BGR2RGB);
        ImageFrame frame;
        frame.src_img = std::move(frame_bgr);
        frame.timestamp = std::chrono::steady_clock::now();

        if (on_frame_callback_) {
            on_frame_callback_(frame);
        }

        return true;
    }
    ImageFrame VideoPlayer::readImage() {
        ImageFrame frame;
        if (!trigger_mode_ || !cap_.isOpened()) {
            return frame;
        }

        cv::Mat frame_bgr;
        cap_ >> frame_bgr;

        if (frame_bgr.empty()) {
            if (loop_) {
                cap_.set(cv::CAP_PROP_POS_FRAMES, start_frame_);
                cap_ >> frame_bgr;
                if (frame_bgr.empty())
                    return frame;
            } else {
                return frame;
            }
        }
        if (use_cvt_)
            cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_BGR2RGB);
        frame.src_img = std::move(frame_bgr);
        frame.timestamp = std::chrono::steady_clock::now();

        if (on_frame_callback_) {
            on_frame_callback_(frame);
        }

        return frame;
    }

    void VideoPlayer::stop() {
        running_ = false;
        if (worker_.joinable()) {
            worker_.join();
        }
        cap_.release();
        WUST_INFO("video") << "Video closed successfully: " << path_;
    }

    VideoPlayer::~VideoPlayer() {
        stop();
    }

    void VideoPlayer::run() {
        using clock = std::chrono::steady_clock;

        const auto frame_interval = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(1.0 / frame_rate_)
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
            if (use_cvt_)
                cv::cvtColor(frame_bgr, frame_bgr, cv::COLOR_BGR2RGB);

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
} // namespace video
} // namespace wust_vl