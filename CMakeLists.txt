cmake_minimum_required(VERSION 3.15)
project(wust_vl LANGUAGES CXX)

# ===============================
# C++ 标准
# ===============================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===============================
# 构建类型与编译选项
# ===============================
set(CMAKE_BUILD_TYPE RELEASE)
add_compile_options(-O3 -w)

# ===============================
# 后端选项
# ===============================
option(BUILD_WITH_TRT      "Enable TensorRT backend"   ON)
option(BUILD_WITH_OPENVINO "Enable OpenVINO backend"   ON)
option(BUILD_WITH_NCNN     "Enable NCNN backend"       ON)
option(BUILD_WITH_ORT      "Enable ORT backend"        ON)

# ===============================
# 版本号
# ===============================
set(WUST_VL_VERSION_MAJOR 1)
set(WUST_VL_VERSION_MINOR 0)
set(WUST_VL_VERSION_PATCH 0)
set(WUST_VL_VERSION "${WUST_VL_VERSION_MAJOR}.${WUST_VL_VERSION_MINOR}.${WUST_VL_VERSION_PATCH}")
add_definitions(-DWUST_VL_VERSION="${WUST_VL_VERSION}")

configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/VersionConfig.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/VersionConfig.h
)

# ===============================
# 依赖库
# ===============================
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

if(BUILD_WITH_OPENVINO)
    find_package(OpenVINO REQUIRED COMPONENTS Runtime ONNX)
endif()

if(BUILD_WITH_TRT)
    find_package(CUDAToolkit REQUIRED)
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/hy/TensorRT-8.5.2.2")
    find_package(TensorRT REQUIRED)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 14)
endif()

if(BUILD_WITH_NCNN)
    find_package(ncnn REQUIRED)
endif()

if(BUILD_WITH_ORT)
    set(ort_root_path "/home/hy/onnxruntime-linux-x64-gpu-1.22.0")
    find_package(Ort REQUIRED)
endif()

# ===============================
# 子模块
# ===============================
add_subdirectory(core/ml_net)
add_subdirectory(core/common)
set(
        TARGETS_LIST
            wust_vl_ml_net
    )
# ===============================
# 安装头文件
# ===============================
install(DIRECTORY include/ DESTINATION include/wust_vl)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/VersionConfig.h DESTINATION include/wust_vl)

install(
    TARGETS
        ${TARGETS_LIST}
    EXPORT
        wust_vlTargets
    RUNTIME DESTINATION
        bin
    LIBRARY DESTINATION
        lib
    ARCHIVE DESTINATION
        lib
)

install(EXPORT wust_vlTargets
    FILE wust_vlTargets.cmake
    NAMESPACE wust_vl::
    DESTINATION lib/cmake/wust_vl
)

# ===============================
# 配置包文件
# ===============================
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/WustVLConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/wust_vlConfig.cmake
    INSTALL_DESTINATION lib/cmake/wust_vl
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/wust_vlConfig.cmake DESTINATION lib/cmake/wust_vl)

# ===============================
# 卸载规则
# ===============================
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY
    )
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    )
endif()
