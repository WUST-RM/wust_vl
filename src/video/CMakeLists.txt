add_library(wust_vl_video SHARED)
include_directories(include)
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
    ${HikSDK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(wust_vl_video
    ${OpenCV_LIBS}
    ${HikSDK_LIB}     
)
if(BUILD_WITH_TRT)
    target_compile_definitions(wust_vl_video PRIVATE USE_TRT)
    target_link_libraries(wust_vl_video 
    wust_vl_cuda
)
endif()

