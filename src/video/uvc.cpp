#include "video/uvc.hpp"
#include <atomic>
#include <chrono>
#include <thread>

namespace wust_vl {
namespace video {

    bool UVC::loadConfig(const YAML::Node& config) {
        WUST_INFO("UVC") << "try get device_name by ls -l /dev/v4l/by-id/";
        try {
            device_name_ = config["device_name"].as<std::string>();

            fps_ = config["fps"].as<int>();

            width_ = config["width"].as<int>();

            height_ = config["height"].as<int>();

            exposure_ = config["exposure"].as<double>();

            gain_ = config["gain"].as<double>();

            gamma_ = config["gamma"].as<double>();

            trigger_mode_ = config["trigger_mode"].as<bool>();

            WUST_INFO("UVC") << "UVC loaded: " << device_name_ << " " << width_ << "x" << height_
                             << "@" << fps_;

            return true;
        } catch (const std::exception& e) {
            WUST_ERROR("UVC") << "Failed to load config: " << e.what();
            return false;
        }
    }

    void UVC::setFrameCallback(FrameCallback cb) {
        on_frame_callback_ = std::move(cb);
    }

    void UVC::start() {
        if (running_) {
            WUST_WARN("UVC") << "Camera already running.";
            return;
        }

        WUST_INFO("UVC") << "Starting camera: " << device_name_;

        if (!cap_.open(device_name_)) {
            WUST_ERROR("UVC") << "Failed to open camera.";
            return;
        }
        double mode = cap_.get(cv::CAP_PROP_AUTO_EXPOSURE);
        WUST_DEBUG("UVC") << "auto exposure mode = " << mode;
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_EXPOSURE, exposure_);
        cap_.set(cv::CAP_PROP_GAIN, gain_);
        cap_.set(cv::CAP_PROP_GAMMA, gamma_);
        cap_.set(cv::CAP_PROP_FPS, fps_);

        running_ = true;

        if (!trigger_mode_) {
            capture_thread_ = std::thread(&UVC::captureLoop, this);
        }

        watchdog_thread_ = std::thread(&UVC::watchdogLoop, this);

        WUST_INFO("UVC") << "Camera started.";
    }

    void UVC::captureLoop() {
        WUST_DEBUG("UVC") << "Capture loop started.";

        while (running_) {
            cv::Mat frame;

            if (!cap_.read(frame)) {
                WUST_WARN("UVC") << "Frame read failed.";
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            if (!on_frame_callback_) {
                WUST_WARN("UVC") << "Frame callback not set.";
                continue;
            }

            ImageFrame image_frame;
            image_frame.src_img = frame;
            image_frame.timestamp = std::chrono::steady_clock::now();
            image_frame.pixel_format = PixelFormat::RGB;
            on_frame_callback_(image_frame);
            last_frame_time_ = std::chrono::steady_clock::now();
        }

        WUST_DEBUG("UVC") << "Capture loop exited.";
    }

    void UVC::watchdogLoop() {
        WUST_DEBUG("UVC") << "Watchdog loop started.";

        while (running_) {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            auto now = std::chrono::steady_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::seconds>(now - last_frame_time_).count();

            if (!trigger_mode_ && duration > 3) {
                WUST_WARN("UVC") << "No frame received for " << duration
                                 << " seconds. Restarting camera.";

                restart();
            }
        }

        WUST_DEBUG("UVC") << "Watchdog loop exited.";
    }

    void UVC::restart() {
        WUST_WARN("UVC") << "Restarting camera.";

        cap_.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!cap_.open(device_name_)) {
            WUST_ERROR("UVC") << "Camera reopen failed.";
            return;
        }

        WUST_INFO("UVC") << "Camera reopened successfully.";
    }

    void UVC::stop() {
        if (!running_)
            return;

        WUST_MAIN("UVC") << "Stopping camera.";

        running_ = false;

        if (capture_thread_.joinable())
            capture_thread_.join();

        if (watchdog_thread_.joinable())
            watchdog_thread_.join();

        cap_.release();

        WUST_INFO("UVC") << "Camera stopped.";
    }

    bool UVC::read() {
        if (!trigger_mode_ || !cap_.isOpened()) {
            WUST_WARN("UVC") << "Manual read invalid state.";
            return false;
        }

        cv::Mat frame;

        if (!cap_.read(frame)) {
            WUST_WARN("UVC") << "Manual read failed.";
            return false;
        }

        if (on_frame_callback_) {
            ImageFrame image_frame;
            image_frame.src_img = frame;
            image_frame.timestamp = std::chrono::steady_clock::now();
            image_frame.pixel_format = PixelFormat::RGB;
            on_frame_callback_(image_frame);
        }

        return true;
    }

    ImageFrame UVC::readImage() {
        ImageFrame frame;

        if (!trigger_mode_ || !cap_.isOpened()) {
            WUST_WARN("UVC") << "readImage invalid state.";
            return frame;
        }

        cv::Mat mat;

        if (cap_.read(mat)) {
            frame.src_img = mat;
            frame.timestamp = std::chrono::steady_clock::now();
            frame.pixel_format = PixelFormat::RGB;
            if (on_frame_callback_)
                on_frame_callback_(frame);
        } else {
            WUST_WARN("UVC") << "readImage failed.";
        }

        return frame;
    }

} // namespace video
} // namespace wust_vl
