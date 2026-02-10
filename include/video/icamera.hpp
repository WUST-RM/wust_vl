#pragma once
#include <chrono>
#include <functional>
#include <opencv2/core/mat.hpp>
#include <yaml-cpp/node/node.h>
namespace wust_vl {
namespace video {
    struct ImageFrame {
        cv::Mat src_img;
        std::chrono::steady_clock::time_point timestamp;
    };

    class ICameraDevice {
    public:
        virtual ~ICameraDevice() = default;

        virtual void start() = 0;
        virtual void stop() = 0;
        virtual bool read() = 0;
        virtual ImageFrame readImage() = 0;
        using FrameCallback = std::function<void(ImageFrame&)>;
        virtual void setFrameCallback(FrameCallback cb) = 0;

        virtual bool loadConfig(const YAML::Node& cfg) = 0;
    };

} // namespace video
} // namespace wust_vl
