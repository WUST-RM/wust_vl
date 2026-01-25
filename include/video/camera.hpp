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
#include "common/utils/logger.hpp"
#include "hik.hpp"
#include "video_player.hpp"
namespace wust_vl {
namespace video {
    enum class CameraType : uint8_t { HIK, VIDEO_PLAYER };
    inline std::string toUpper(const std::string& s) {
        std::string res = s;
        std::transform(res.begin(), res.end(), res.begin(), [](unsigned char c) {
            return std::toupper(c);
        });
        return res;
    }
    inline CameraType string2CameraType(const std::string& str) {
        std::string s = toUpper(str);
        if (s == "HIK_CAMERA")
            return CameraType::HIK;
        else if (s == "VIDEO_PLAYER")
            return CameraType::VIDEO_PLAYER;
        else
            return CameraType::HIK;
    }
    class Camera {
    public:
        bool init(const YAML::Node& config) {
            config_ = config;
            type_ = string2CameraType(config["type"].as<std::string>());
            switch (type_) {
                case CameraType::HIK:
                    device_ = std::make_shared<HikCamera>();
                    device_->loadConfig(config["hik_camera"]);
                    break;

                case CameraType::VIDEO_PLAYER:
                    device_ = std::make_shared<VideoPlayer>();
                    device_->loadConfig(config["video_player"]);
                    break;
            }
            WUST_INFO("camera") << "init camera success";
            return true;
        }
        void read() {
            if (device_)
                device_->read();
        }
        ImageFrame readImage() {
            if (device_)
                return device_->readImage();
            return {};
        }
        void start() {
            if (device_)
                device_->start();
        }
        void stop() {
            if (device_)
                device_->stop();
        }

        void setFrameCallback(ICameraDevice::FrameCallback cb) {
            if (device_)
                device_->setFrameCallback(cb);
        }

        ICameraDevice* getDevice() const {
            return device_.get();
        }

        CameraType type_;
        std::shared_ptr<ICameraDevice> device_;
        YAML::Node config_;
    };
} // namespace video
} // namespace wust_vl