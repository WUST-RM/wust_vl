# FindTensorRT.cmake -- Locate NVIDIA TensorRT

if (NOT CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
endif()

include(FindPackageHandleStandardArgs)

# 支持设置 TensorRT_ROOT 提供自定义路径
if (DEFINED TensorRT_ROOT)
  list(APPEND _TensorRT_SEARCH_PATHS
    ${TensorRT_ROOT}
    "$ENV{TensorRT_ROOT}"
  )
endif()
list(APPEND _TensorRT_SEARCH_PATHS /usr /usr/local)

# 查找头文件
find_path(TensorRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES include
)
# 查找核心库
find_library(TensorRT_LIBRARY
  NAMES nvinfer 
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES lib lib64 lib/x64
)

# 检查是否找到
find_package_handle_standard_args(TensorRT
  REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_LIBRARY
)

if (TensorRT_FOUND)
  set(TensorRT_INCLUDE_DIRS ${TensorRT_INCLUDE_DIR})
  set(TensorRT_LIBRARIES ${TensorRT_LIBRARY})

  # 可选插件或解析器
  foreach(_comp IN ITEMS nvinfer_plugin nvonnxparser nvparsers)
    find_library(TensorRT_${_comp}_LIBRARY
      NAMES ${_comp}
      PATHS ${_TensorRT_SEARCH_PATHS}
      PATH_SUFFIXES lib lib64 lib/x64
    )
    if (TensorRT_${_comp}_LIBRARY)
      list(APPEND TensorRT_LIBRARIES ${TensorRT_${_comp}_LIBRARY})
    endif()
  endforeach()

  # 导出 IMPORTED target
  add_library(TensorRT::TensorRT UNKNOWN IMPORTED)
  set_target_properties(TensorRT::TensorRT PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${TensorRT_LIBRARY}"
  )
  # 若有插件，分别创建
  foreach(_comp IN ITEMS nvinfer_plugin nvonnxparser nvparsers)
    if (TARGET TensorRT::${_comp})
      add_library(TensorRT::${_comp} UNKNOWN IMPORTED)
      set_target_properties(TensorRT::${_comp} PROPERTIES
        IMPORTED_LOCATION "${TensorRT_${_comp}_LIBRARY}"
      )
      list(APPEND TensorRT_LIBRARIES TensorRT::${_comp})
    endif()
  endforeach()

  message(STATUS "Found TensorRT ${TensorRT_VERSION_STRING} at ${TensorRT_INCLUDE_DIR}")
endif()
