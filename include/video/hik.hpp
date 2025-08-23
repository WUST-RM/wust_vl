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
#include "common/utils/recorder.hpp"
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
    void startCamera(bool if_recorder);
    bool restartCamera();
    void stopCamera();
    bool enableTrigger(TriggerType type, const std::string& source, int64_t activation);
    void disableTrigger();
    bool read();
    ImageFrame readImage();

private:
    void hikCaptureLoop();

    void* camera_handle_;
    int fail_count_;
    MV_IMAGE_BASIC_INFO img_info_;
    MV_CC_PIXEL_CONVERT_PARAM convert_param_;
    std::thread capture_thread_;
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
    std::unique_ptr<Recorder> recorder_;
};
} // namespace wust_vl_video