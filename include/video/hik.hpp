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
#include "common/utils/logger.hpp"
#include "icamera.hpp"
#include <thread>
#include <yaml-cpp/yaml.h>
namespace wust_vl {
namespace video {

    class HikCamera: public ICameraDevice {
    public:
        HikCamera();
        ~HikCamera();
        enum class TriggerType { None, Software, Hardware };
        inline TriggerType string2TriggerType(const std::string& str) {
            static const std::unordered_map<std::string, TriggerType> lut = {
                { "none", TriggerType::None },
                { "software", TriggerType::Software },
                { "hardware", TriggerType::Hardware }
            };

            std::string key = str;
            std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) {
                return std::tolower(c);
            });

            auto it = lut.find(key);
            return (it != lut.end()) ? it->second : TriggerType::None;
        }

        // enum -> string
        inline std::string triggerType2String(TriggerType type) {
            switch (type) {
                case TriggerType::None:
                    return "None";
                case TriggerType::Software:
                    return "Software";
                case TriggerType::Hardware:
                    return "Hardware";
            }
            return "None";
        }
#define HIK_SET_FLOAT_RANGE(camera_handle, param, val) \
    do { \
        MVCC_FLOATVALUE _fv {}; \
        int _s = MV_CC_GetFloatValue(camera_handle, param, &_fv); \
        if (_s == MV_OK) { \
            double _c = std::clamp((double)val, (double)_fv.fMin, (double)_fv.fMax); \
            int _r = MV_CC_SetFloatValue(camera_handle, param, _c); \
            if (_r == MV_OK) \
                WUST_INFO("hik_camera") << param << " set to " << _c; \
            else \
                WUST_ERROR("hik_camera") << "Failed to set " << param << ", status=" << _r; \
        } else { \
            WUST_ERROR("hik_camera") << "Failed to get " << param << " range, status=" << _s; \
        } \
    } while (0)
#define HIK_SET_INT_RANGE(camera_handle, param, val) \
    do { \
        MVCC_INTVALUE _iv {}; \
        int _s = MV_CC_GetIntValue(camera_handle, param, &_iv); \
        if (_s == MV_OK) { \
            int64_t _c = std::clamp((int64_t)val, (int64_t)_iv.nMin, (int64_t)_iv.nMax); \
            int _r = MV_CC_SetIntValue(camera_handle, param, _c); \
            if (_r == MV_OK) { \
                WUST_INFO("hik_camera") << param << " set to " << _c; \
            } else { \
                WUST_ERROR("hik_camera") << "Failed to set " << param << ", status=" << _r; \
            } \
        } else { \
            WUST_ERROR("hik_camera") << "Failed to get " << param << " range, status=" << _s; \
        } \
    } while (0)
#define HIK_SET_BOOL(camera_handle, param, val) \
    do { \
        int _r = MV_CC_SetBoolValue(camera_handle, param, (bool)(val)); \
        if (_r == MV_OK) { \
            WUST_INFO("hik_camera") << param << " set to " << ((val) ? 1 : 0); \
        } else { \
            WUST_ERROR("hik_camera") << "Failed to set " << param << ", status=" << _r; \
        } \
    } while (0)
#define HIK_SET_ENUM_STR(camera_handle, param, val_str) \
    do { \
        int _r = MV_CC_SetEnumValueByString(camera_handle, param, (val_str).c_str()); \
        if (_r == MV_OK) { \
            WUST_INFO("hik_camera") << param << " set to " << (val_str); \
        } else { \
            WUST_ERROR("hik_camera") << "Failed to set " << param << ", status=" << _r; \
        } \
    } while (0)
        template<typename T>
        inline void HikSetRangeDispatch(void* camera_handle, const char* param, const T& val) {
            if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
                HIK_SET_INT_RANGE(camera_handle, param, val);
            } else if constexpr (std::is_floating_point_v<T>) {
                HIK_SET_FLOAT_RANGE(camera_handle, param, val);
            } else if constexpr (std::is_same_v<T, bool>) {
                HIK_SET_BOOL(camera_handle, param, val);
            } else if constexpr (std::is_same_v<T, std::string>) {
                HIK_SET_ENUM_STR(camera_handle, param, val);
            } else {
                static_assert(sizeof(T) == 0, "Unsupported HikCamera param type");
            }
        }
#define HIK_GEN_MEMBER_GET_SET(type, camera_handle, param) \
    type param##_val {}; \
    inline void set##param(const type& v) { \
        param##_val = v; \
        HikSetRangeDispatch(camera_handle, #param, param##_val); \
    } \
    inline type get##param() const { \
        return param##_val; \
    }

        bool loadConfig(const YAML::Node& config) override;
        void setFrameCallback(FrameCallback cb) override {
            on_frame_callback_ = std::move(cb);
        }

        bool initializeCamera(const std::string& target_sn);
        void start() override;
        bool restart();
        void stop() override;
        bool setTrigger(TriggerType type, const std::string& source, int64_t activation);
        void disableTrigger();
        bool read() override;
        ImageFrame readImage() override;
        HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, PixelFormat)
        HIK_GEN_MEMBER_GET_SET(std::string, camera_handle_, ADCBitDepth)
        HIK_GEN_MEMBER_GET_SET(bool, camera_handle_, ReverseX)
        HIK_GEN_MEMBER_GET_SET(bool, camera_handle_, ReverseY)
        HIK_GEN_MEMBER_GET_SET(int, camera_handle_, Width)
        HIK_GEN_MEMBER_GET_SET(int, camera_handle_, Height)
        HIK_GEN_MEMBER_GET_SET(int, camera_handle_, OffsetX)
        HIK_GEN_MEMBER_GET_SET(int, camera_handle_, OffsetY)
        HIK_GEN_MEMBER_GET_SET(bool, camera_handle_, AcquisitionFrameRateEnable)
        HIK_GEN_MEMBER_GET_SET(double, camera_handle_, AcquisitionFrameRate)
        HIK_GEN_MEMBER_GET_SET(double, camera_handle_, Gain)
        HIK_GEN_MEMBER_GET_SET(double, camera_handle_, Gamma)
        HIK_GEN_MEMBER_GET_SET(double, camera_handle_, ExposureTime)
    private:
        void hikCaptureLoop(wust_vl::common::concurrency::MonitoredThread::Ptr self);
        YAML::Node config_;
        bool use_rgb_ = false;
        bool use_ea_ = false;
        bool use_raw_ = false;
        void* camera_handle_;
        int fail_count_;
        wust_vl::common::concurrency::MonitoredThread::Ptr capture_thread_;
        std::string hik_logger_ = "hik_camera";
        std::string target_sn_;
        bool in_low_frame_rate_state_;
        TriggerType trigger_type_ = TriggerType::None;
        std::string trigger_source_;
        int64_t trigger_activation_;
        std::atomic<bool> stop_signal_ { false };
        int video_fps_;
        int expected_width_ = 0;
        int expected_height_ = 0;
        FrameCallback on_frame_callback_;
        wust_vl::common::concurrency::TimedQueue<ImageFrame> img_queue_ { 1.0 }; // 1s 有效时间窗口
        std::chrono::steady_clock::time_point low_frame_rate_start_time_;
        wust_vl::common::concurrency::MonitoredThread::Ptr process_thread_;
        void hikProcessLoop(wust_vl::common::concurrency::MonitoredThread::Ptr self);
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
} // namespace video
} // namespace wust_vl