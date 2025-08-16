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
#include "openvino/openvino.hpp"
namespace ml_net {
class OpenvinoNet {
public:
    using PppinitFun = std::function<void(ov::preprocess::PrePostProcessor&)>;
    struct Params {
        std::string model_path;
        std::string device_name;
        ov::hint::PerformanceMode mode;
    };
    OpenvinoNet();
    ~OpenvinoNet();
    bool init(const Params& params, PppinitFun ppp_init_fun);
    ov::Tensor infer(const ov::Tensor& input_tensor);
    std::pair<ov::element::Type, ov::Shape> getInputInfo();

private:
    Params params_;
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
} // namespace ml_net