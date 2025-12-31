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

#include "MvCameraControl.h"
#include "common/concurrency/monitored_thread.hpp"
#include "common/concurrency/queues.hpp"
#include "video/image.hpp"
#include <thread>
#include <yaml-cpp/yaml.h>
namespace wust_vl_video {
enum class TriggerType { None, Software, Hardware };
class HikCamera {
public:
    HikCamera();
    ~HikCamera();
    bool init(const YAML::Node& config);
    void setFrameCallback(std::function<void(ImageFrame&)> cb) {
        on_frame_callback_ = std::move(cb);
    }

    bool initializeCamera(const std::string& target_sn);
    void setParameters(
        double acquisition_frame_rate,
        double exposure_time,
        double gain,
        double gamma,
        const std::string& adc_bit_depth,
        const std::string& pixel_format,
        bool acquisition_frame_rate_enable,
        bool reverse_x,
        bool reverse_y
    );
    void setExposureTime(double exposure_time);
    double getExposureTime() const {
        return last_exposure_time_;
    }
    void startCamera();
    bool restartCamera();
    void stopCamera();
    bool enableTrigger(TriggerType type, const std::string& source, int64_t activation);
    void disableTrigger();
    bool read();
    ImageFrame readImage();
    void setPixelFormat(const std::string& pixel_format);
    void setRgb(bool rgb) {
        use_rgb_ = rgb;
    }
    void setEA(bool ea) {
        use_ea_ = ea;
    }

private:
    void hikCaptureLoop(std::shared_ptr<wust_vl_concurrency::MonitoredThread> self);
    bool use_rgb_ = false;
    bool use_ea_ = false;
    void* camera_handle_;
    int fail_count_;
    MV_IMAGE_BASIC_INFO img_info_;
    MV_CC_PIXEL_CONVERT_PARAM convert_param_;
    std::shared_ptr<wust_vl_concurrency::MonitoredThread> capture_thread_;
    std::string hik_logger_ = "hik_camera";
    double last_frame_rate_, last_exposure_time_, last_gain_, last_gamma_;
    bool last_acquisition_frame_rate_enable_;
    std::string last_adc_bit_depth_, last_pixel_format_;
    std::string last_target_sn_;
    bool in_low_frame_rate_state_;
    bool last_reverse_x_, last_reverse_y_;
    TriggerType trigger_type_ = TriggerType::None;
    std::string trigger_source_; // e.g. "Line0"、"Software"
    int64_t trigger_activation_; // 0=FallingEdge, 1=RisingEdge
    std::chrono::steady_clock::time_point low_frame_rate_start_time_;
    std::atomic<bool> stop_signal_ { false };
    int video_fps_;
    int expected_width_ = 0;
    int expected_height_ = 0;
    std::function<void(ImageFrame&)> on_frame_callback_;
    TimedQueue<ImageFrame> img_queue_ { 1.0 }; // 1s 有效时间窗口
    std::shared_ptr<wust_vl_concurrency::MonitoredThread> process_thread_;
    void hikProcessLoop(std::shared_ptr<wust_vl_concurrency::MonitoredThread> self);
    const std::unordered_map<MvGvspPixelType, int> PIXEL_MAP_RGB_EA = {
        { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB_EA },
        { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB_EA },
        { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB_EA },
        { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB_EA },
        { PixelType_Gvsp_RGB8_Packed, -1 },
        { PixelType_Gvsp_Mono8, cv::COLOR_GRAY2RGB },
    };
    const std::unordered_map<MvGvspPixelType, int> PIXEL_MAP_RGB = {
        { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB },
        { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB },
        { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB },
        { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB },
        { PixelType_Gvsp_RGB8_Packed, -1 },
        { PixelType_Gvsp_Mono8, cv::COLOR_GRAY2RGB },
    };
    const std::unordered_map<MvGvspPixelType, int> PIXEL_MAP_BGR_EA = {
        { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR_EA },
        { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR_EA },
        { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR_EA },
        { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR_EA },
        { PixelType_Gvsp_RGB8_Packed, -1 },
        { PixelType_Gvsp_Mono8, cv::COLOR_GRAY2BGR },
    };
    const std::unordered_map<MvGvspPixelType, int> PIXEL_MAP_BGR = {
        { PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR },
        { PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR },
        { PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR },
        { PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR },
        { PixelType_Gvsp_RGB8_Packed, -1 },
        { PixelType_Gvsp_Mono8, cv::COLOR_GRAY2BGR },
    };
    const std::unordered_map<MvGvspPixelType, int> img_type_map = {
        { PixelType_Gvsp_BayerGR8, CV_8UC1 },    { PixelType_Gvsp_BayerRG8, CV_8UC1 },
        { PixelType_Gvsp_BayerGB8, CV_8UC1 },    { PixelType_Gvsp_BayerBG8, CV_8UC1 },
        { PixelType_Gvsp_RGB8_Packed, CV_8UC3 }, { PixelType_Gvsp_Mono8, CV_8UC1 },
    };
};
} // namespace wust_vl_video