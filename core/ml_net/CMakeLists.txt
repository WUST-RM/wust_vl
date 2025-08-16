cmake_minimum_required(VERSION 3.14)
project(ml_net LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(ML_NET_SOURCES
    # 根据启用的后端加入对应源文件
)

if(BUILD_WITH_OPENVINO)
    list(APPEND ML_NET_SOURCES src/openvino/openvino_net.cpp)
endif()
if(BUILD_WITH_TRT)
    list(APPEND ML_NET_SOURCES src/tensorrt/tensorrt_net.cpp)
endif()
if(BUILD_WITH_ORT)
    list(APPEND ML_NET_SOURCES src/onnxruntime/onnxruntime_net.cpp)
endif()
if(BUILD_WITH_NCNN)
    list(APPEND ML_NET_SOURCES src/ncnn/ncnn_net.cpp)
endif()

add_library(ml_net SHARED ${ML_NET_SOURCES})

target_include_directories(ml_net PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
)

if(BUILD_WITH_OPENVINO)
    target_link_libraries(ml_net PUBLIC openvino::runtime openvino::frontend::onnx)
    target_compile_definitions(ml_net PUBLIC USE_OPENVINO)
endif()
if(BUILD_WITH_TRT)
    target_include_directories(ml_net PUBLIC ${TensorRT_INCLUDE_DIR})
    target_link_libraries(ml_net PUBLIC TensorRT::TensorRT CUDA::cudart)
    target_compile_definitions(ml_net PUBLIC USE_TRT)
endif()
if(BUILD_WITH_ORT)
    target_include_directories(ml_net PUBLIC ${Ort_INCLUDE_DIR})
    target_link_libraries(ml_net PUBLIC ${Ort_LIB})
    target_compile_definitions(ml_net PUBLIC USE_ORT)
endif()
if(BUILD_WITH_NCNN)
    target_include_directories(ml_net PUBLIC ${ncnn_INCLUDE_DIRS})
    target_link_libraries(ml_net PUBLIC ncnn)
    target_compile_definitions(ml_net PUBLIC USE_NCNN)
endif()

# 安装 ml_net 到 wust_vl 路径下
install(TARGETS ml_net
    EXPORT wust_vlTargets
    LIBRARY DESTINATION lib/wust_vl
    ARCHIVE DESTINATION lib/wust_vl
    RUNTIME DESTINATION bin/wust_vl
)

install(EXPORT ml_netTargets
    FILE ml_netTargets.cmake
    NAMESPACE wust_vl::
    DESTINATION lib/cmake/wust_vl
)

install(DIRECTORY include/ DESTINATION include/wust_vl)
