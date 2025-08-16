#include "ml_net/openvino/openvino_net.hpp"
namespace ml_net {

struct OpenvinoNet::Impl {
    Impl() = default;
    ~Impl() {
        compiled_model_.reset();
        ov_core_.reset();
        model_.reset();
    }
    void init(const Params& params, PppinitFun ppp_init_fun) {
        params_ = params;
        if (!ov_core_) {
            ov_core_ = std::make_unique<ov::Core>();
        }
        model_ = ov_core_->read_model(params_.model_path);
        ov::preprocess::PrePostProcessor ppp(model_);
        ppp_init_fun(ppp);
        model_ = ppp.build();
        compiled_model_ = std::make_unique<ov::CompiledModel>(ov_core_->compile_model(
            model_,
            params_.device_name,
            ov::hint::performance_mode(params_.mode)
        ));
    }
    ov::Tensor infer(const ov::Tensor& input_tensor) {
        auto infer_request = compiled_model_->create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();
        return infer_request.get_output_tensor();
    }
    std::pair<ov::element::Type, ov::Shape> getInputInfo() {
        auto input_port = compiled_model_->input();
        return { input_port.get_element_type(), input_port.get_shape() };
    }
    std::unique_ptr<ov::Core> ov_core_;
    std::unique_ptr<ov::CompiledModel> compiled_model_;
    std::shared_ptr<ov::Model> model_;
    Params params_;
};
OpenvinoNet::OpenvinoNet(): _impl(std::make_unique<Impl>()) {}
OpenvinoNet::~OpenvinoNet() {
    _impl.reset();
}
bool OpenvinoNet::init(const Params& params, PppinitFun ppp_init_fun) {
    _impl->init(params, ppp_init_fun);
    return true;
}
ov::Tensor OpenvinoNet::infer(const ov::Tensor& input_tensor) {
    return _impl->infer(input_tensor);
}
std::pair<ov::element::Type, ov::Shape> OpenvinoNet::getInputInfo() {
    return _impl->getInputInfo();
}
} // namespace ml_net