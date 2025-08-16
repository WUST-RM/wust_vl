#include "ml_net/tensorrt/tensorrt_net.hpp"
#include "NvOnnxParser.h"
#include "fmt/color.h"
#include "fmt/core.h"
#include "fmt/printf.h"
#include <fstream>
#include <iostream>
#include <vector>
namespace ml_net {
#define TRT_ASSERT(expr) \
    do { \
        if (!(expr)) { \
            fmt::print(fmt::fg(fmt::color::red), "assert fail: '" #expr "'"); \
            exit(-1); \
        } \
    } while (0)
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error: " << cudaGetErrorString(err) << " at " << __FILE__ << ":" \
                      << __LINE__ << std::endl; \
            return nullptr; \
        } \
    } while (0)

struct TensorRTNet::Impl {
    Impl() = default;
    ~Impl() {
        delete[] output_buffer_;
        cudaStreamDestroy(stream_);
        cudaFree(device_buffers_[output_idx_]);
        cudaFree(device_buffers_[input_idx_]);

        if (context_)
            context_->destroy();
        if (engine_)
            engine_->destroy();
        if (runtime_)
            runtime_->destroy();
    }
    bool init(const Params& params) {
        params_ = params;
        buildEngine(params_.model_path);
        TRT_ASSERT(context_ = engine_->createExecutionContext());
        TRT_ASSERT((input_idx_ = engine_->getBindingIndex("images")) == 0);
        TRT_ASSERT((output_idx_ = engine_->getBindingIndex("output")) == 1);
        input_dims_ = engine_->getBindingDimensions(input_idx_);
        output_dims_ = engine_->getBindingDimensions(output_idx_);
        input_sz_ = input_dims_.d[1] * input_dims_.d[2] * input_dims_.d[3];
        output_sz_ = output_dims_.d[1] * output_dims_.d[2];
        TRT_ASSERT(cudaMalloc(&device_buffers_[input_idx_], input_sz_ * sizeof(float)) == 0);
        TRT_ASSERT(cudaMalloc(&device_buffers_[output_idx_], output_sz_ * sizeof(float)) == 0);
        output_buffer_ = new float[output_sz_];
        TRT_ASSERT(cudaStreamCreate(&stream_) == 0);
        return true;
    }
    void buildEngine(const std::string& onnx_path) {
        std::string engine_path = onnx_path.substr(0, onnx_path.find_last_of('.')) + ".engine";
        std::ifstream engine_file(engine_path, std::ios::binary);
        if (engine_file.good()) {
            engine_file.seekg(0, std::ios::end);
            size_t size = engine_file.tellg();
            engine_file.seekg(0, std::ios::beg);
            std::vector<char> engine_data(size);
            engine_file.read(engine_data.data(), size);
            engine_file.close();

            runtime_ = nvinfer1::createInferRuntime(g_logger_); // ✅ 作为成员变量保存
            engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
            if (engine_ != nullptr) {
                std::cout << "Load engine from " << engine_path << " successfully." << std::endl;
                return;
            }
        }
        std::cout << "building new engine..." << std::endl;
        // 构建新引擎
        auto builder = nvinfer1::createInferBuilder(g_logger_);
        const auto explicit_batch = 1U
            << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        auto network = builder->createNetworkV2(explicit_batch);
        auto parser = nvonnxparser::createParser(*network, g_logger_);
        parser->parseFromFile(
            onnx_path.c_str(),
            static_cast<int>(nvinfer1::ILogger::Severity::kINFO)
        );

        auto config = builder->createBuilderConfig();
        if (builder->platformHasFastFp16()) {
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
        }
        engine_ = builder->buildEngineWithConfig(*network, *config);

        // 保存引擎
        auto serialized_engine = engine_->serialize();
        std::ofstream out_file(engine_path, std::ios::binary);
        out_file.write(
            reinterpret_cast<const char*>(serialized_engine->data()),
            serialized_engine->size()
        );
        out_file.close();
        serialized_engine->destroy();

        // 反序列化仍然需要 runtime_
        if (!runtime_) {
            runtime_ = nvinfer1::createInferRuntime(g_logger_);
        }

        // 清理
        parser->destroy();
        network->destroy();
        config->destroy();
        builder->destroy();
    }
    nvinfer1::IExecutionContext* getAContext() {
        return engine_->createExecutionContext();
    }
    cudaStream_t getStream() {
        return stream_;
    }
    void input2Device(void* input_data) {
        // 将输入数据复制到设备内存
        TRT_ASSERT(
            cudaMemcpyAsync(
                device_buffers_[input_idx_],
                input_data,
                input_sz_ * sizeof(float),
                cudaMemcpyHostToDevice,
                stream_
            )
            == 0
        );
    }
    float* output2Host() {
        // 从设备内存复制输出数据到主机内存
        TRT_ASSERT(
            cudaMemcpyAsync(
                output_buffer_,
                device_buffers_[output_idx_],
                output_sz_ * sizeof(float),
                cudaMemcpyDeviceToHost,
                stream_
            )
            == 0
        );
        TRT_ASSERT(cudaStreamSynchronize(stream_) == 0);
        return output_buffer_;
    }
    void* getDeviceOutput() {
        return device_buffers_[output_idx_];
    }
    void* getInputTensorPtr() {
        return device_buffers_[input_idx_];
    }

    void infer(void* input_data, nvinfer1::IExecutionContext* context) {
        context->setTensorAddress("images", input_data);
        context->setTensorAddress("output", device_buffers_[output_idx_]);

        if (!context->enqueueV3(stream_)) {
            std::cerr << "enqueueV3 failed!";
            return;
        }
    }
    std::tuple<nvinfer1::Dims, nvinfer1::Dims> getInputOutputDims() {
        return { input_dims_, output_dims_ };
    }
    Params params_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IExecutionContext* context_;
    nvinfer1::IRuntime* runtime_ = nullptr;
    void* device_buffers_[2]; // 输入输出显存指针
    float* output_buffer_; // 输出数据主机内存
    TRTLogger g_logger_;
    int input_idx_, output_idx_;
    size_t input_sz_, output_sz_;
    nvinfer1::Dims input_dims_;
    nvinfer1::Dims output_dims_;
    cudaStream_t stream_;
};
TensorRTNet::TensorRTNet(): _impl(std::make_unique<Impl>()) {}
TensorRTNet::~TensorRTNet() {
    _impl.reset();
}
bool TensorRTNet::init(const Params& params) {
    return _impl->init(params);
}
nvinfer1::IExecutionContext* TensorRTNet::getAContext() {
    return _impl->getAContext();
}
cudaStream_t TensorRTNet::getStream() {
    return _impl->getStream();
}
void TensorRTNet::input2Device(void* input_data) {
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
void TensorRTNet::infer(void* input_data, nvinfer1::IExecutionContext* context) {
    _impl->infer(input_data, context);
}
std::tuple<nvinfer1::Dims, nvinfer1::Dims> TensorRTNet::getInputOutputDims() {
    return _impl->getInputOutputDims();
}
} // namespace ml_net