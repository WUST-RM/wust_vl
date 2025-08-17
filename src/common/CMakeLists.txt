add_library(wust_vl_common SHARED)
target_sources(
    wust_vl_common
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/common.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers/recorder.cpp
)
target_include_directories(wust_vl_common PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
    ${OpenCV_INCLUDE_DIRS}
)
