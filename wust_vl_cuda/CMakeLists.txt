cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0079 NEW)

project(wust_vl_cuda LANGUAGES CXX CUDA)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wno-deprecated-declarations)

set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_OBJECTS OFF)
set(CMAKE_CUDA_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE CUDA_INFER_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cu
)

add_library(wust_vl_cuda SHARED ${CUDA_INFER_SRC})
set_target_properties(wust_vl_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)

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

target_link_libraries(wust_vl_cuda PUBLIC
    ${CUDA_NPP_LIBRARIES}
    nppicc
    ${OpenCV_LIBS}
    CUDA::cudart
    TensorRT::TensorRT
    
)
