#pragma once
#include <chrono>
#include <functional>
#include <opencv2/core/mat.hpp>
#include <yaml-cpp/node/node.h>
namespace wust_vl {
namespace video {
    enum class PixelFormat : uint8_t {
        UNKNOWN = 0,
        GRAY = 1,
        BGR = 2,
        RGB = 3,
    };
    struct ImageFrame {
        cv::Mat src_img;
        std::chrono::steady_clock::time_point timestamp;
        PixelFormat pixel_format = PixelFormat::UNKNOWN;
        ImageFrame() = default;

        // 2) 拷贝构造
        ImageFrame(const ImageFrame& other) noexcept:
            src_img(other.src_img),
            timestamp(other.timestamp),
            pixel_format(other.pixel_format) {}
        ImageFrame& operator=(const ImageFrame& other) noexcept {
            if (this != &other) {
                src_img = other.src_img;
                timestamp = other.timestamp;
                pixel_format = other.pixel_format;
            }
            return *this;
        }
        ImageFrame& operator=(ImageFrame&& other) noexcept {
            if (this != &other) {
                src_img = std::move(other.src_img);
                timestamp = other.timestamp;
                pixel_format = other.pixel_format;
            }
            return *this;
        }
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
