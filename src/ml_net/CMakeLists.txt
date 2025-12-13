add_library(wust_vl_ml_net SHARED)

target_sources(wust_vl_ml_net
    PRIVATE
    $<$<BOOL:${BUILD_WITH_OPENVINO}>:${CMAKE_CURRENT_SOURCE_DIR}/openvino/openvino_net.cpp>
    $<$<BOOL:${BUILD_WITH_TRT}>:${CMAKE_CURRENT_SOURCE_DIR}/tensorrt/tensorrt_net.cpp>
    $<$<BOOL:${BUILD_WITH_ORT}>:${CMAKE_CURRENT_SOURCE_DIR}/onnxruntime/onnxruntime_net.cpp>
    $<$<BOOL:${BUILD_WITH_NCNN}>:${CMAKE_CURRENT_SOURCE_DIR}/ncnn/ncnn_net.cpp>
)

target_include_directories(wust_vl_ml_net
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
)

if(BUILD_WITH_OPENVINO)
    target_link_libraries(wust_vl_ml_net PRIVATE openvino::runtime openvino::frontend::onnx)
    target_compile_definitions(wust_vl_ml_net PRIVATE USE_OPENVINO)
endif()

if(BUILD_WITH_TRT)
    target_link_libraries(wust_vl_ml_net PRIVATE TensorRT::TensorRT TensorRT::nvonnxparser CUDA::cudart)
    target_compile_definitions(wust_vl_ml_net PRIVATE USE_TRT)
endif()

if(BUILD_WITH_ORT)
    target_link_libraries(wust_vl_ml_net PRIVATE ${Ort_LIBS})
    target_compile_definitions(wust_vl_ml_net PRIVATE USE_ORT)
endif()

if(BUILD_WITH_NCNN)
    target_link_libraries(wust_vl_ml_net PRIVATE ncnn)
    target_compile_definitions(wust_vl_ml_net PRIVATE USE_NCNN)
endif()



