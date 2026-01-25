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
#pragma once
#include <onnxruntime_cxx_api.h>
namespace wust_vl {

namespace ml_net {
    enum class OrtProvider : uint8_t {
        CPU, //!< 由 `CPU` 执行
        CUDA, //!< 由 `CUDA` 执行
        TensorRT, //!< 由 `TensorRT` 执行
        OpenVINO //!< 由 `OpenVINO` 执行
    };
    inline OrtProvider string2OrtProvider(const std::string& str) {
        std::string s = str;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);

        if (s == "cpu")
            return OrtProvider::CPU;
        if (s == "cuda")
            return OrtProvider::CUDA;
        if (s == "tensorrt")
            return OrtProvider::TensorRT;
        if (s == "openvino")
            return OrtProvider::OpenVINO;

        throw std::invalid_argument("Invalid OrtProvider string: " + str);
    }
    class OnnxRuntimeNet {
    public:
        struct Params {
            std::string model_path;
            OrtProvider provider;
        };
        OnnxRuntimeNet();
        ~OnnxRuntimeNet();

        void init(const Params& params);
        float* infer(float* input_data, size_t input_size);
        std::vector<int64_t> getOutputShape();

    private:
        Params params_;
        struct Impl;
        std::unique_ptr<Impl> _impl;
    };
} // namespace ml_net
} // namespace wust_vl