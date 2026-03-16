#pragma once
#include "../common/utils/logger.hpp"
#include "icamera.hpp"
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

namespace wust_vl {
namespace video {
    class UVC: public ICameraDevice {
    public:
        ~UVC();
        bool loadConfig(const YAML::Node& config) override;

        void setFrameCallback(FrameCallback cb) override;
        void start() override;
        void stop() override;
        bool read() override;
        ImageFrame readImage() override;
        void captureLoop();
        void watchdogLoop();
        void restart();
        std::thread capture_thread_;
        std::thread watchdog_thread_;
        std::string device_name_;
        int fps_;
        int width_;
        int height_;
        double exposure_;
        double gain_;
        double gamma_;
        bool trigger_mode_ = false;
        bool running_ = false;
        cv::VideoCapture cap_;
        FrameCallback on_frame_callback_;
        std::chrono::steady_clock::time_point last_frame_time_;
    };

} // namespace video
} // namespace wust_vl
