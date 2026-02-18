add_library(wust_vl_video SHARED)
target_sources(
    wust_vl_video
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/hik.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/uvc.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/video_player.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/camera.cpp

)
find_package(HikSDK REQUIRED)
target_include_directories(wust_vl_video PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
)
target_link_libraries(wust_vl_video
    HikSDK::HikSDK
    ${OpenCV_LIBS}
)
if(BUILD_WITH_TRT)
    target_compile_definitions(wust_vl_video PRIVATE USE_TRT)
    target_link_libraries(wust_vl_video 
    wust_vl_cuda
)
endif()

