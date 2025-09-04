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
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include <iostream>
#include <memory>
#include <string>

namespace ml_net {
class TRTLogger: public nvinfer1::ILogger {
public:
    explicit TRTLogger(
        nvinfer1::ILogger::Severity severity = nvinfer1::ILogger::Severity::kWARNING
    ):
        severity_(severity) {}
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override {
        if (severity <= severity_) {
            std::cerr << msg << std::endl;
        }
    }
    nvinfer1::ILogger::Severity severity_;
};
class TensorRTNet {
public:
    struct Params {
        std::string model_path;
        int input_w;
        int input_h;
    };
    TensorRTNet();
    ~TensorRTNet();
    bool init(const Params& params);
    nvinfer1::IExecutionContext* getAContext();
    cudaStream_t getStream();
    void input2Device(void* input_data);
    float* output2Host();
    void* getDeviceOutput();
    void* getInputTensorPtr();
    void infer(void* input_data, nvinfer1::IExecutionContext* context);
    std::tuple<nvinfer1::Dims, nvinfer1::Dims> getInputOutputDims();
    Params params_;
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
} // namespace ml_net