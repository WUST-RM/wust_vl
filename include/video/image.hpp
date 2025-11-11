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

#include "opencv2/opencv.hpp"
namespace wust_vl_video {

struct ImageFrame {
    std::vector<uint8_t> data;
    int width;
    int height;
    int step;
    int pixel_type;
    int img_type;
    cv::Mat src_img;
    std::chrono::steady_clock::time_point timestamp;
};

inline cv::Mat convertToMat(const ImageFrame& frame, bool use_raw = false) {
    if (frame.data.empty()) {
        return cv::Mat();
    }
    cv::Mat bayer_img(
        frame.height,
        frame.width,
        frame.img_type,
        const_cast<uint8_t*>(frame.data.data()),
        frame.step
    );
    cv::Mat img;
    if (use_raw || frame.pixel_type < 0) {
        img = bayer_img.clone();
    } else {
        cv::cvtColor(bayer_img, img, frame.pixel_type);
    }

    return img;
}
} // namespace wust_vl_video