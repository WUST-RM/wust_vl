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
#include "video/hik.hpp"
#include "common/utils/logger.hpp"
#include <pwd.h>

namespace wust_vl_video {
inline void changeFileOwner(const std::string& filepath, const std::string& username) {
    struct passwd* pwd = getpwnam(username.c_str());
    if (pwd == nullptr) {
        perror("getpwnam failed");
        return;
    }
    uid_t uid = pwd->pw_uid;
    gid_t gid = pwd->pw_gid;

    if (chown(filepath.c_str(), uid, gid) != 0) {
        perror("chown failed");
    }
}
inline std::string getOriginalUsername() {
    const char* sudo_user = std::getenv("SUDO_USER");
    if (sudo_user) {
        return std::string(sudo_user);
    }
    uid_t uid = getuid();
    struct passwd* pw = getpwuid(uid);
    if (pw) {
        return std::string(pw->pw_name);
    }
    return "";
}
HikCamera::HikCamera(): camera_handle_(nullptr), fail_count_(0) {}

HikCamera::~HikCamera() {
    stop();

    if (capture_thread_) {
        capture_thread_->stop();
        wust_vl_concurrency::ThreadManager::instance().unregisterThread(capture_thread_->getName());
    }
    if (process_thread_) {
        process_thread_->stop();
        wust_vl_concurrency::ThreadManager::instance().unregisterThread(process_thread_->getName());
    }
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(&camera_handle_);
    }

    WUST_INFO(hik_logger_) << "Camera destroyed!";
}
bool HikCamera::loadConfig(const YAML::Node& config) {
    config_ = config;
    if (capture_thread_) {
        capture_thread_->heartbeat();
    }
    std::string target_sn = config["target_sn"].as<std::string>();
    if (!initializeCamera(target_sn)) {
        WUST_ERROR("hik_camera") << "Hik Camera initialization failed.";
        return false;
    }

    auto acquisition_frame_rate = config["acquisition_frame_rate"].as<double>();
    setAcquisitionFrameRate(acquisition_frame_rate);
    auto exposure_time = config["exposure_time"].as<double>();
    setExposureTime(exposure_time);
    auto gain = config["gain"].as<double>();
    setGain(gain);
    auto gamma = config["gamma"].as<double>();
    setGamma(gamma);
    auto adc_bit_depth = config["adc_bit_depth"].as<std::string>();
    setADCBitDepth(adc_bit_depth);
    auto pixel_format = config["pixel_format"].as<std::string>();
    setPixelFormat(pixel_format);
    auto acquisition_frame_rate_enable = config["acquisition_frame_rate_enable"].as<bool>();
    setAcquisitionFrameRateEnable(acquisition_frame_rate_enable);
    auto width = config["width"].as<int>();
    setWidth(width);
    auto height = config["height"].as<int>();
    setHeight(height);
    auto offset_x = config["offset_x"].as<int>();
    setOffsetX(offset_x);
    auto offset_y = config["offset_y"].as<int>();
    setOffsetY(offset_y);
    auto reverse_x = config["reverse_x"].as<bool>();
    setReverseX(reverse_x);
    auto reverse_y = config["reverse_y"].as<bool>();
    setReverseY(reverse_y);

    std::string trigger_type_str = config["trigger_type"].as<std::string>();
    trigger_type_ = string2TriggerType(trigger_type_str);
    trigger_source_ = config["trigger_source"].as<std::string>();
    trigger_activation_ = config["trigger_activation"].as<int64_t>();
    setTrigger(trigger_type_, trigger_source_, trigger_activation_);
    use_rgb_ = config["use_rgb"].as<bool>();
    use_ea_ = config["use_ea"].as<bool>();
    use_raw_ = config["use_raw"].as<bool>();
    WUST_INFO(hik_logger_) << "Camera parameters set successfully!";

    return true;
}
bool HikCamera::initializeCamera(const std::string& target_sn) {
    target_sn_ = target_sn;

    if (camera_handle_ != nullptr) {
        WUST_INFO(hik_logger_) << "Closing previously opened camera";
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }

    while (!stop_signal_) {
        if (capture_thread_) {
            capture_thread_->heartbeat();
        }

        MV_CC_DEVICE_INFO_LIST device_list = { 0 };
        int n_ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (n_ret != MV_OK) {
            WUST_ERROR(hik_logger_) << "MV_CC_EnumDevices failed, error code: " << n_ret;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (device_list.nDeviceNum == 0) {
            WUST_ERROR(hik_logger_) << "No USB cameras found";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        WUST_INFO(hik_logger_) << "Found " << device_list.nDeviceNum << " USB camera(s):";
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            auto info = device_list.pDeviceInfo[i];
            const char* sn =
                reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            WUST_INFO(hik_logger_) << "  [" << i << "] SN = " << sn;
        }

        int sel = -1;
        for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
            auto info = device_list.pDeviceInfo[i];
            const char* sn =
                reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
            if (target_sn == sn) {
                sel = i;
                break;
            }
        }

        if (sel < 0) {
            WUST_ERROR(hik_logger_) << "Camera with serial " << target_sn << " not found";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        WUST_INFO(hik_logger_) << "Selecting camera at index " << sel << " (SN=" << target_sn
                               << ")";

        n_ret = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[sel]);
        if (n_ret != MV_OK) {
            WUST_ERROR(hik_logger_) << "MV_CC_CreateHandle failed: " << n_ret;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        n_ret = MV_CC_OpenDevice(camera_handle_);
        if (n_ret != MV_OK) {
            WUST_ERROR(hik_logger_) << "MV_CC_OpenDevice failed: " << n_ret;
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        disableTrigger();

        WUST_INFO(hik_logger_) << "Camera initialized successfully";
        return true;
    }
    return false;
}

bool HikCamera::setTrigger(TriggerType type, const std::string& source, int64_t activation) {
    trigger_type_ = type;
    trigger_source_ = source;
    trigger_activation_ = activation;
    if (type != TriggerType::None) {
        if (MV_CC_SetEnumValueByString(camera_handle_, "TriggerMode", "On") != MV_OK)
            return false;
        if (MV_CC_SetEnumValueByString(camera_handle_, "TriggerSource", source.c_str()) != MV_OK)
            return false;

        const char* act = (activation == 1 ? "RisingEdge" : "FallingEdge");
        if (MV_CC_SetEnumValueByString(camera_handle_, "TriggerActivation", act) != MV_OK)
            return false;

        WUST_INFO(hik_logger_) << "Trigger enabled: src=" << source << ", act=" << act;
    }

    return true;
}

void HikCamera::disableTrigger() {
    trigger_type_ = TriggerType::None;
    MV_CC_SetEnumValueByString(camera_handle_, "TriggerMode", "Off");
    WUST_INFO(hik_logger_) << "Trigger disabled, continuous mode.";
}

void HikCamera::start() {
    int n_ret = MV_CC_StartGrabbing(camera_handle_);
    if (n_ret != MV_OK) {
        WUST_ERROR(hik_logger_) << "Failed to start camera grabbing!";
    }
    MVCC_INTVALUE stParam = { 0 };
    if (MV_CC_GetIntValue(camera_handle_, "Width", &stParam) == MV_OK) {
        expected_width_ = stParam.nCurValue;
    }
    if (MV_CC_GetIntValue(camera_handle_, "Height", &stParam) == MV_OK) {
        expected_height_ = stParam.nCurValue;
    }
    if (trigger_type_ != TriggerType::Software) {
        capture_thread_ = wust_vl_concurrency::MonitoredThread::create(
            "HikCaptureThread",
            [this](std::shared_ptr<wust_vl_concurrency::MonitoredThread> self) {
                this->hikCaptureLoop(self);
            }
        );

        wust_vl_concurrency::ThreadManager::instance().registerThread(capture_thread_);
        process_thread_ = wust_vl_concurrency::MonitoredThread::create(
            "HikProcessThread",
            [this](std::shared_ptr<wust_vl_concurrency::MonitoredThread> self) {
                this->hikProcessLoop(self);
            }
        );
        wust_vl_concurrency::ThreadManager::instance().registerThread(process_thread_);
    }
}

bool HikCamera::restart() {
    WUST_WARN(hik_logger_) << "Restarting camera from scratch...";
    if (capture_thread_) {
        capture_thread_->heartbeat();
    }
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    loadConfig(config_);

    int n_ret = MV_CC_StartGrabbing(camera_handle_);
    if (n_ret != MV_OK) {
        WUST_ERROR(hik_logger_) << "Failed to start grabbing after restart.";
        return false;
    }

    WUST_INFO(hik_logger_) << "Camera restarted successfully!";
    return true;
}
void HikCamera::hikProcessLoop(std::shared_ptr<wust_vl_concurrency::MonitoredThread> self) {
    while (self->isAlive()) {
        ImageFrame frame;
        self->heartbeat();
        if (img_queue_.pop_valid(frame)) {
            if (on_frame_callback_) {
                frame.src_img = convertToMat(frame, use_raw_);
                on_frame_callback_(frame);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}
void HikCamera::hikCaptureLoop(std::shared_ptr<wust_vl_concurrency::MonitoredThread> self) {
    MV_FRAME_OUT out_frame;
    WUST_INFO(hik_logger_) << "Starting image capture loop!";

    auto fail_start_time = std::chrono::steady_clock::now();
    bool in_fail_state = false;

    in_low_frame_rate_state_ = false;
    auto last_frame_rate_check = std::chrono::steady_clock::now();
    int frame_counter = 0;

    try {
        while (self->isAlive()) {
            self->heartbeat();
            auto start_time = std::chrono::steady_clock::now();
            int n_ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 100);
            if (n_ret == MV_OK) {
                in_fail_state = false;
                ++frame_counter;

                ImageFrame frame;
                auto current_time = std::chrono::steady_clock::now();

                auto half_exposure = std::chrono::microseconds((long)(getExposureTime() / 2));
                frame.timestamp = current_time - half_exposure;
                frame.width = out_frame.stFrameInfo.nWidth;
                frame.height = out_frame.stFrameInfo.nHeight;
                frame.data.resize(out_frame.stFrameInfo.nFrameLen);
                std::memcpy(frame.data.data(), out_frame.pBufAddr, out_frame.stFrameInfo.nFrameLen);
                const auto& frame_info = out_frame.stFrameInfo;
                auto pixel_type = frame_info.enPixelType;

                frame.img_type = img_type_map.at(pixel_type);
                if (frame.img_type == CV_8UC3) {
                    frame.step = frame.width * 3;
                } else if (frame.img_type == CV_8UC1) {
                    frame.step = frame.width;
                }
                const auto& map_ref = use_rgb_ ? (use_ea_ ? PIXEL_MAP_RGB_EA : PIXEL_MAP_RGB)
                                               : (use_ea_ ? PIXEL_MAP_BGR_EA : PIXEL_MAP_BGR);
                frame.pixel_type = map_ref.at(pixel_type);
                img_queue_.push(std::move(frame));

                MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

                if (std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - last_frame_rate_check
                    )
                        .count()
                    >= 1)
                {
                    float actual_fps = static_cast<float>(frame_counter);
                    frame_counter = 0;
                    last_frame_rate_check = current_time;

                    if (actual_fps < getAcquisitionFrameRate() * 0.5f) {
                        if (!in_low_frame_rate_state_) {
                            low_frame_rate_start_time_ = current_time;
                            in_low_frame_rate_state_ = true;
                            WUST_WARN(hik_logger_) << "Low FPS detected: " << actual_fps;
                        } else if (std::chrono::duration_cast<std::chrono::seconds>(
                           current_time - low_frame_rate_start_time_)
                           .count() >= 5)
                        {
                            WUST_ERROR(hik_logger_)
                                << "Low FPS persisted for 5s. Restarting camera...";
                            if (restart()) {
                                in_low_frame_rate_state_ = false;
                                WUST_INFO(hik_logger_) << "Camera restarted successfully";
                            } else {
                                WUST_ERROR(hik_logger_) << "Restart failed, exiting capture loop.";
                                break;
                            }
                        }
                    } else if (in_low_frame_rate_state_) {
                        in_low_frame_rate_state_ = false;
                        WUST_INFO(hik_logger_) << "FPS recovered to normal: " << actual_fps;
                    }
                }

            } else {
                if (!in_fail_state) {
                    fail_start_time = std::chrono::steady_clock::now();
                    in_fail_state = true;
                }

                if (std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - fail_start_time
                    )
                        .count()
                    > 5)
                {
                    if (!restart()) {
                        WUST_ERROR(hik_logger_)
                            << "Failed to restart camera after hardware failure.";
                        std::exit(EXIT_FAILURE);
                        break;
                    }
                    fail_start_time = std::chrono::steady_clock::now();
                    in_fail_state = false;
                }
            }
        }
    } catch (const std::exception& e) {
        WUST_ERROR(hik_logger_) << "Exception in capture loop: " << e.what();
        stop_signal_ = true;
    } catch (...) {
        WUST_ERROR(hik_logger_) << "Unknown exception in capture loop!";
        stop_signal_ = true;
    }
    WUST_INFO(hik_logger_) << "Exiting image capture loop.";
}

void HikCamera::stop() {
    stop_signal_ = true;
}
bool HikCamera::read() {
    if (trigger_type_ == TriggerType::None) {
        WUST_WARN(hik_logger_) << "read() called in non-trigger mode. Ignored.";
        return false;
    }

    if (trigger_type_ == TriggerType::Software) {
        MV_CC_SetCommandValue(camera_handle_, "TriggerSoftware");
    }

    MV_FRAME_OUT out_frame;
    int n_ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000); // 1s timeout
    if (n_ret != MV_OK) {
        WUST_ERROR(hik_logger_) << "Failed to get image buffer in read()";
        return false;
    }

    ImageFrame frame;
    frame.width = out_frame.stFrameInfo.nWidth;
    frame.height = out_frame.stFrameInfo.nHeight;
    frame.data.resize(out_frame.stFrameInfo.nFrameLen);
    std::memcpy(frame.data.data(), out_frame.pBufAddr, out_frame.stFrameInfo.nFrameLen);
    const auto& frame_info = out_frame.stFrameInfo;
    auto pixel_type = frame_info.enPixelType;

    frame.img_type = img_type_map.at(pixel_type);
    if (frame.img_type == CV_8UC3) {
        frame.step = frame.width * 3;
    } else if (frame.img_type == CV_8UC1) {
        frame.step = frame.width;
    }
    const auto& map_ref = use_rgb_ ? (use_ea_ ? PIXEL_MAP_RGB_EA : PIXEL_MAP_RGB)
                                   : (use_ea_ ? PIXEL_MAP_BGR_EA : PIXEL_MAP_BGR);
    frame.pixel_type = map_ref.at(pixel_type);
    auto half_exposure = std::chrono::microseconds((long)(getExposureTime() / 2));
    frame.timestamp = std::chrono::steady_clock::now() - half_exposure;

    if (on_frame_callback_) {
        on_frame_callback_(frame);
    }

    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

    return false;
}
ImageFrame HikCamera::readImage() {
    if (trigger_type_ == TriggerType::None) {
        WUST_WARN(hik_logger_) << "read() called in non-trigger mode. Ignored.";
        return ImageFrame();
    }

    if (trigger_type_ == TriggerType::Software) {
        MV_CC_SetCommandValue(camera_handle_, "TriggerSoftware");
    }

    MV_FRAME_OUT out_frame;
    int n_ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000); // 1s timeout
    if (n_ret != MV_OK) {
        WUST_ERROR(hik_logger_) << "Failed to get image buffer in read()";
        return ImageFrame();
    }

    ImageFrame frame;
    frame.width = out_frame.stFrameInfo.nWidth;
    frame.height = out_frame.stFrameInfo.nHeight;
    frame.data.resize(out_frame.stFrameInfo.nFrameLen);
    std::memcpy(frame.data.data(), out_frame.pBufAddr, out_frame.stFrameInfo.nFrameLen);
    const auto& frame_info = out_frame.stFrameInfo;
    auto pixel_type = frame_info.enPixelType;
    frame.img_type = img_type_map.at(pixel_type);
    if (frame.img_type == CV_8UC3) {
        frame.step = frame.width * 3;
    } else if (frame.img_type == CV_8UC1) {
        frame.step = frame.width;
    }
    const auto& map_ref = use_rgb_ ? (use_ea_ ? PIXEL_MAP_RGB_EA : PIXEL_MAP_RGB)
                                   : (use_ea_ ? PIXEL_MAP_BGR_EA : PIXEL_MAP_BGR);
    frame.pixel_type = map_ref.at(pixel_type);
    auto half_exposure = std::chrono::microseconds((long)(getExposureTime() / 2));
    frame.timestamp = std::chrono::steady_clock::now() - half_exposure;

    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
    return frame;
}
} // namespace wust_vl_video