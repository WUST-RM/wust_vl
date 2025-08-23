add_library(wust_vl_video SHARED)
include_directories(include)
target_sources(
    wust_vl_video
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/hik.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/video_player.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/camera.cpp

)
find_package(HikSDK REQUIRED)
target_include_directories(wust_vl_video PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
    ${HikSDK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

