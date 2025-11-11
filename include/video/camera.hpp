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
#include "hik.hpp"
#include "video_player.hpp"
namespace wust_vl_video {
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
    Camera();
    ~Camera();
    bool init(const YAML::Node& config);
    void start();
    void stop();
    void read();
    ImageFrame readImage();
    void enableHikTrigger(TriggerType type, const std::string& source, int64_t activation);
    void setHikExposureTime(double exposure_time);
    void setHikRgb(bool rgb);
    double getHikExposureTime() const;
    void setFrameCallback(std::function<void(ImageFrame&)> cb);
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
} // namespace wust_vl_video