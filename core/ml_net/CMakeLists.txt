cmake_minimum_required(VERSION 3.14)

# -------------------
# 创建库
# -------------------
add_library(wust_vl_ml_net SHARED)

# -------------------
# 收集源文件
# -------------------
target_sources(wust_vl_ml_net
    PRIVATE
    $<$<BOOL:${BUILD_WITH_OPENVINO}>:${CMAKE_CURRENT_SOURCE_DIR}/src/openvino/openvino_net.cpp>
    $<$<BOOL:${BUILD_WITH_TRT}>:${CMAKE_CURRENT_SOURCE_DIR}/src/tensorrt/tensorrt_net.cpp>
    $<$<BOOL:${BUILD_WITH_ORT}>:${CMAKE_CURRENT_SOURCE_DIR}/src/onnxruntime/onnxruntime_net.cpp>
    $<$<BOOL:${BUILD_WITH_ORT}>:${CMAKE_CURRENT_SOURCE_DIR}/src/ncnn/ncnn_net.cpp>
)

# -------------------
# include 路径
# -------------------
target_include_directories(wust_vl_ml_net
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
)

# -------------------
# 后端依赖
# -------------------
if(BUILD_WITH_OPENVINO)
    target_link_libraries(wust_vl_ml_net PUBLIC openvino::runtime openvino::frontend::onnx)
    target_compile_definitions(wust_vl_ml_net PUBLIC USE_OPENVINO)
endif()
if(BUILD_WITH_TRT)
    target_link_libraries(wust_vl_ml_net PUBLIC TensorRT::TensorRT CUDA::cudart)
    target_compile_definitions(wust_vl_ml_net PUBLIC USE_TRT)
endif()
if(BUILD_WITH_ORT)
    target_link_libraries(wust_vl_ml_net PUBLIC ${Ort_LIBS})   
    target_compile_definitions(wust_vl_ml_net PUBLIC USE_ORT)
endif()
if(BUILD_WITH_NCNN)
    target_link_libraries(wust_vl_ml_net PUBLIC ncnn)
    target_compile_definitions(wust_vl_ml_net PUBLIC USE_NCNN)
endif()
