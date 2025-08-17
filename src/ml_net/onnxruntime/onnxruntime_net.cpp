#include "ml_net/onnxruntime/onnxruntime_net.hpp"

namespace ml_net {
struct OnnxRuntimeNet::Impl {
    Impl() = default;
    ~Impl() {
        env_.reset();
        session_.reset();
    }
    void init(const Params& params) {
        params_ = params;

        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNX");

        // 2. 配置 SessionOptions
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(4);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // 3. 选择执行提供器
        switch (params_.provider) {
            case OrtProvider::CUDA:
                session_options.AppendExecutionProvider_CUDA({});
                break;
            case OrtProvider::TensorRT:
                session_options.AppendExecutionProvider_TensorRT({});
                break;
            case OrtProvider::OpenVINO: {
                OrtOpenVINOProviderOptions options;
                options.device_type = "CPU_FP32"; // 可改为 MYRIAD / GPU_FP32 等
                session_options.AppendExecutionProvider_OpenVINO(options);
                break;
            }
            case OrtProvider::CPU:
            default:
                // 默认使用 CPU
                break;
        }

        // 4. 创建推理 Session
        session_ =
            std::make_unique<Ort::Session>(*env_, params_.model_path.c_str(), session_options);

        // 5. 分配默认内存管理器
        Ort::AllocatorWithDefaultOptions allocator;

        // 6. 获取输入节点名称
        input_name_ = session_->GetInputNameAllocated(0, allocator).get();

        // 7. 获取输入张量形状
        auto input_type_info = session_->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_dims_ = input_tensor_info.GetShape();

        // 8. 获取输出节点名称
        output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
    }
    float* infer(std::vector<float> input_tensor_values) {
        Ort::MemoryInfo memory_info =
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            input_tensor_values.data(),
            input_tensor_values.size(),
            input_dims_.data(),
            input_dims_.size()
        );

        const char* input_names[] = { input_name_.c_str() };
        const char* output_names[] = { output_name_.c_str() };
        auto output_tensors =
            session_
                ->Run(Ort::RunOptions { nullptr }, input_names, &input_tensor, 1, output_names, 1);

        float* output_data = output_tensors.front().GetTensorMutableData<float>();
        return output_data;
    }
    std::vector<int64_t> getOutputShape() {
        auto output_type_info = session_->GetOutputTypeInfo(0);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        return output_tensor_info.GetShape();
    }

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    Ort::SessionOptions session_options_;
    Params params_;
    std::vector<int64_t> input_dims_;
    std::string input_name_;
    std::string output_name_;
};
OnnxRuntimeNet::OnnxRuntimeNet(): _impl(std::make_unique<Impl>()) {}
OnnxRuntimeNet::~OnnxRuntimeNet() {
    _impl.reset();
}
void OnnxRuntimeNet::init(const Params& params) {
    _impl->init(params);
}
float* OnnxRuntimeNet::infer(std::vector<float> input_tensor_values) {
    return _impl->infer(input_tensor_values);
}
std::vector<int64_t> OnnxRuntimeNet::getOutputShape() {
    return _impl->getOutputShape();
}
} // namespace ml_net