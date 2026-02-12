cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0079 NEW)

project(wust_vl_cuda LANGUAGES CXX CUDA)

# 设置标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_BUILD_TYPE "Release")
# 抑制过时 API 警告
add_compile_options(-Wno-deprecated-declarations)

# 禁用 .rsp 响应文件（避免 nvcc 报错）
set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_OBJECTS OFF)
set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

# 查找依赖
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 收集源码
file(GLOB_RECURSE CUDA_INFER_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cu
)

# 添加静态库
add_library(wust_vl_cuda STATIC ${CUDA_INFER_SRC})
set_target_properties(wust_vl_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 设置包含路径
target_include_directories(wust_vl_cuda
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CUDAToolkit_INCLUDE_DIRS}
        ${TensorRT_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


# 设置 CUDA 编译选项
target_compile_options(wust_vl_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --generate-code=arch=compute_86,code=sm_86
        -Xcompiler=-fPIC
        -O3
        -w
        -Wno-deprecated-gpu-targets
        -Wno-error=deprecated-declarations
    >
)

# 链接库
target_link_libraries(wust_vl_cuda PRIVATE
    ${OpenCV_LIBS}
    CUDA::cudart
    TensorRT::TensorRT
)
