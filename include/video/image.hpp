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
namespace wust_vl {
namespace video {
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

    inline cv::Mat convertToMat(ImageFrame& frame, bool use_raw = false) {
        if (frame.data.empty() || frame.width <= 0 || frame.height <= 0 || frame.step <= 0) {
            return cv::Mat();
        }

        cv::Mat src(frame.height, frame.width, frame.img_type, frame.data.data(), frame.step);
        CV_Assert(src.step * src.rows <= frame.data.size());
        CV_Assert(src.cols * src.elemSize() <= src.step);

        if (use_raw) {
            return src.clone();
        }
        cv::Mat out;
        if (frame.pixel_type >= 0) {
            CV_Assert(src.type() == CV_8UC1);
            cv::cvtColor(src, out, frame.pixel_type);
            return out;
        }
        if (src.type() == CV_8UC1) {
            cv::cvtColor(src, out, cv::COLOR_GRAY2RGB);
            return out;
        }
        if (src.type() == CV_8UC3) {
            return src.clone();
        }
        return src.clone();
    }

} // namespace video
} // namespace wust_vl