
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
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
namespace wust_vl {
namespace common {
    namespace utils {
        class Labeler {
        public:
            Labeler();
            ~Labeler();

            void save(const cv::Mat& image, const std::vector<float>& data_row);

        private:
            std::string image_dir_;
            std::string csv_dir_;
            std::string temp_csv_path_;

            std::ofstream csv_file_;
            int image_index_ = 1;
            int start_index_ = 1;
            bool csv_header_written_ = false;

            int getLastImageIndex() const;
        };
    } // namespace utils
} // namespace common
} // namespace wust_vl