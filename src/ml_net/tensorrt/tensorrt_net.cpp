#include "ml_net/tensorrt/tensorrt_net.hpp"
#include "NvOnnxParser.h"

#include <cuda_runtime.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

namespace wust_vl::ml_net {

#define TRT_CHECK(expr) \
    do { \
        const auto _ret = (expr); \
        if (_ret != cudaSuccess) { \
            std::cerr << "\033[31mCUDA error: " << cudaGetErrorString(_ret) << " (" << #expr \
                      << ")\033[0m\n"; \
            std::abort(); \
        } \
    } while (0)
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
struct TensorRTNet::Impl {
    Impl() = default;
    ~Impl() {
        release();
    }

    void release() noexcept {
        if (stream_)
            cudaStreamDestroy(stream_);
        if (device_buffers_[0])
            cudaFree(device_buffers_[0]);
        if (device_buffers_[1])
            cudaFree(device_buffers_[1]);

        context_.reset();
        engine_.reset();
        runtime_.reset();
    }

    static int64_t volume(const nvinfer1::Dims& dims) {
        int64_t v = 1;
        for (int i = 0; i < dims.nbDims; ++i)
            v *= dims.d[i];
        return v;
    }

    bool init(const Params& params) {
        params_ = params;

        buildEngine(params_.model_path);

        context_.reset(engine_->createExecutionContext());
        if (!context_)
            throw std::runtime_error("Failed to create execution context");

        input_name_ = engine_->getIOTensorName(0);
        output_name_ = engine_->getIOTensorName(1);
        std::cout<<"input_name_: "<<input_name_<<" output_name_: "<<output_name_<<std::endl;
        input_idx_ = 0;
        output_idx_ = 1;

        context_->setInputShape(input_name_, params_.input_dims);
        if (!context_->allInputShapesSpecified())
            throw std::runtime_error("Input shape not specified");

        input_dims_ = context_->getTensorShape(input_name_);
        output_dims_ = context_->getTensorShape(output_name_);

        input_sz_ = volume(input_dims_);
        output_sz_ = volume(output_dims_);

        TRT_CHECK(cudaMalloc(&device_buffers_[input_idx_], input_sz_ * sizeof(float)));
        TRT_CHECK(cudaMalloc(&device_buffers_[output_idx_], output_sz_ * sizeof(float)));

        output_buffer_.resize(output_sz_);

        TRT_CHECK(cudaStreamCreate(&stream_));
        return true;
    }

    void buildEngine(const std::string& onnx_path) {
        const std::string engine_path =
            onnx_path.substr(0, onnx_path.find_last_of('.')) + ".engine";

        runtime_.reset(nvinfer1::createInferRuntime(g_logger_));

        {
            std::ifstream fin(engine_path, std::ios::binary);
            if (fin.good()) {
                fin.seekg(0, std::ios::end);
                const size_t size = fin.tellg();
                fin.seekg(0, std::ios::beg);

                std::vector<char> data(size);
                fin.read(data.data(), size);

                engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
                if (engine_) {
                    std::cout << "Loaded TensorRT engine: " << engine_path << '\n';
                    return;
                }
                std::cerr << "Engine load failed, rebuilding...\n";
            }
        }

        std::cout << "Building TensorRT engine from ONNX...\n";

        auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(g_logger_));

        const uint32_t flags =
            1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

        auto network =
            std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flags));

        auto parser =
            std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, g_logger_));

        if (!parser->parseFromFile(
                onnx_path.c_str(),
                static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)
            ))
        {
            throw std::runtime_error("Failed to parse ONNX: " + onnx_path);
        }

        auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());

        if (builder->platformHasFastFp16())
            config->setFlag(nvinfer1::BuilderFlag::kFP16);

        auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
            builder->buildSerializedNetwork(*network, *config)
        );

        engine_.reset(runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));

        if (!engine_)
            throw std::runtime_error("Engine build failed");

        std::ofstream fout(engine_path, std::ios::binary);
        fout.write(static_cast<const char*>(serialized->data()), serialized->size());

        std::cout << "Engine built & saved: " << engine_path << '\n';
    }

    void input2Device(const void* host_input) {
        TRT_CHECK(cudaMemcpyAsync(
            device_buffers_[input_idx_],
            host_input,
            input_sz_ * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_
        ));
    }

    float* output2Host() {
        TRT_CHECK(cudaMemcpyAsync(
            output_buffer_.data(),
            device_buffers_[output_idx_],
            output_sz_ * sizeof(float),
            cudaMemcpyDeviceToHost,
            stream_
        ));
        return output_buffer_.data();
    }

    void infer(void* input_device_ptr, nvinfer1::IExecutionContext* ctx) {
        ctx->setTensorAddress(input_name_, input_device_ptr);
        ctx->setTensorAddress(output_name_, device_buffers_[output_idx_]);

        if (!ctx->enqueueV3(stream_))
            throw std::runtime_error("enqueueV3 failed");
    }

    nvinfer1::IExecutionContext* getAContext() {
        return engine_->createExecutionContext();
    }

    cudaStream_t getStream() const {
        return stream_;
    }

    void* getDeviceOutput() const {
        return device_buffers_[output_idx_];
    }
    void* getInputTensorPtr() const {
        return device_buffers_[input_idx_];
    }

    std::tuple<nvinfer1::Dims, nvinfer1::Dims> getInputOutputDims() const {
        return { input_dims_, output_dims_ };
    }

    Params params_;

    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;

    void* device_buffers_[2] { nullptr, nullptr };
    std::vector<float> output_buffer_;

    TRTLogger g_logger_;

    int input_idx_ { 0 }, output_idx_ { 1 };
    size_t input_sz_ { 0 }, output_sz_ { 0 };

    nvinfer1::Dims input_dims_ {};
    nvinfer1::Dims output_dims_ {};

    cudaStream_t stream_ { nullptr };

    const char* input_name_ { nullptr };
    const char* output_name_ { nullptr };
};

TensorRTNet::TensorRTNet(): _impl(std::make_unique<Impl>()) {}

TensorRTNet::~TensorRTNet() = default;

bool TensorRTNet::init(const Params& params) {
    return _impl->init(params);
}

nvinfer1::IExecutionContext* TensorRTNet::getAContext() {
    return _impl->getAContext();
}

cudaStream_t TensorRTNet::getStream() {
    return _impl->getStream();
}

void TensorRTNet::input2Device(const void* input_data) {
    _impl->input2Device(input_data);
}

float* TensorRTNet::output2Host() {
    return _impl->output2Host();
}

void* TensorRTNet::getDeviceOutput() {
    return _impl->getDeviceOutput();
}

void* TensorRTNet::getInputTensorPtr() {
    return _impl->getInputTensorPtr();
}

void TensorRTNet::infer(void* input_data, nvinfer1::IExecutionContext* ctx) {
    _impl->infer(input_data, ctx);
}

std::tuple<nvinfer1::Dims, nvinfer1::Dims> TensorRTNet::getInputOutputDims() {
    return _impl->getInputOutputDims();
}

} // namespace wust_vl::ml_net
