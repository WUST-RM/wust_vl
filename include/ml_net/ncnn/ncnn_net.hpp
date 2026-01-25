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
#include "ncnn/net.h"
#include <memory>
namespace wust_vl {
namespace ml_net {
    class NCNNNet {
    public:
        struct Params {
            std::string model_path_param;
            std::string model_path_bin;
            std::string input_name;
            std::string output_name;
            bool use_vulkan;
            int device_id;
            bool use_light_mode;
            int cpu_threads;
        };
        NCNNNet();
        ~NCNNNet();
        void init(const Params& params);
        ncnn::Mat infer(const ncnn::Mat& input_tensor);

    private:
        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
} // namespace ml_net
} // namespace wust_vl