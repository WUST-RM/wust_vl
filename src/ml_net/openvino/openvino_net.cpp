#include "ml_net/openvino/openvino_net.hpp"
namespace wust_vl {
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
                ov_core_->set_property(params_.device_name, ov::hint::inference_precision("f16"));
            }
            model_ = ov_core_->read_model(params_.model_path);
            ov::preprocess::PrePostProcessor ppp(model_);
            ppp_init_fun(ppp);
            model_ = ppp.build();
            ov::AnyMap config = { { ov::hint::performance_mode.name(), params_.mode },
                                  { ov::hint::inference_precision.name(), "f16" } };
            compiled_model_ = std::make_unique<ov::CompiledModel>(
                ov_core_->compile_model(model_, params_.device_name, config)
            );
        }
        ov::InferRequest createInferRequest() {
            return compiled_model_->create_infer_request();
        }
        ov::Tensor infer(const ov::Tensor& input_tensor, ov::InferRequest& infer_request) {
            infer_request.set_input_tensor(input_tensor);
            infer_request.infer();
            return infer_request.get_output_tensor();
        }
        ov::Tensor infer(const ov::Tensor& input_tensor) {
            auto infer_request = compiled_model_->create_infer_request();

            infer_request.set_input_tensor(input_tensor);
            infer_request.infer();
            return infer_request.get_output_tensor();
        }
        ov::Tensor infer_thread_local(const ov::Tensor& input_tensor) {
            static thread_local std::unique_ptr<ov::InferRequest> infer_request;
            if (!infer_request) {
                infer_request = std::make_unique<ov::InferRequest>(createInferRequest());
            }

            infer_request->set_input_tensor(input_tensor);
            infer_request->infer();
            return infer_request->get_output_tensor();
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
    ov::Tensor OpenvinoNet::infer(const ov::Tensor& input_tensor, ov::InferRequest& infer_request) {
        return OpenvinoNet::_impl->infer(input_tensor, infer_request);
    }
    ov::InferRequest OpenvinoNet::createInferRequest() {
        return OpenvinoNet::_impl->createInferRequest();
    }
    ov::Tensor OpenvinoNet::infer_thread_local(const ov::Tensor& input_tensor) {
        return OpenvinoNet::_impl->infer_thread_local(input_tensor);
    }
} // namespace ml_net
} // namespace wust_vl