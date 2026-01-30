#pragma once
#ifdef USE_NCNN
    #include "ncnn/ncnn_net.hpp"
#endif
#ifdef USE_ORT
    #include "onnxruntime/onnxruntime_net.hpp"
#endif
#ifdef USE_OPENVINO
    #include "openvino/openvino_net.hpp"
#endif
#ifdef USE_TRT
    #include "tensorrt/tensorrt_net.hpp"
#endif
