#include "ml_net/ncnn/ncnn_net.hpp"
#include <iostream>
namespace ml_net {

struct NCNNNet::Impl {
    Impl() = default;
    ~Impl() {
        net_.clear();
    }
    void init(const NCNNNet::Params& params) {
        params_ = params;
        if (params_.use_vulkan) {
            ncnn::create_gpu_instance();
            opt_.use_vulkan_compute = true;
            ncnn::VulkanDevice* vkdev = ncnn::get_gpu_device(params_.device_id);
            if (vkdev) {
                net_.set_vulkan_device(vkdev);
            }
            std::cout << "ncnn: use gpu" << std::endl;
        } else {
            opt_.use_vulkan_compute = false;
            std::cout << "ncnn: use cpu" << std::endl;
        }
        if (params_.use_light_mode) {
            opt_.lightmode = true;
        }

        opt_.num_threads = params_.cpu_threads;
        net_.opt = opt_;

        if (net_.load_param(params_.model_path_param.c_str()) != 0) {
            std::cerr << "Failed to load param" << params_.model_path_param << std::endl;
            return;
        }
        if (net_.load_model(params_.model_path_bin.c_str()) != 0) {
            std::cerr << "Failed to load model" << params_.model_path_bin << std::endl;
            return;
        }

        int ret = net_.load_param((params_.model_path_param).c_str());
        if (ret != 0) {
            std::cerr << "Failed to load param file: " << params_.model_path_param << std::endl;
            return;
        }

        ret = net_.load_model((params_.model_path_bin).c_str());
        if (ret != 0) {
            std::cerr << "Failed to load bin file: " << params_.model_path_bin << std::endl;
            return;
        }
    }
    ncnn::Mat infer(const ncnn::Mat& input) {
        ncnn::Extractor ex = net_.create_extractor();
        ex.input(params_.input_name.c_str(), input);

        ncnn::Mat out;
        ex.extract(params_.output_name.c_str(), out);
        return out;
    }
    Params params_;
    ncnn::Net net_;
    ncnn::Option opt_;
};
NCNNNet::NCNNNet(): _impl(std::make_unique<Impl>()) {}
NCNNNet::~NCNNNet() {
    _impl.reset();
}
void NCNNNet::init(const Params& params) {
    _impl->init(params);
}
ncnn::Mat NCNNNet::infer(const ncnn::Mat& input_tensor) {
    return _impl->infer(input_tensor);
}

} // namespace ml_net