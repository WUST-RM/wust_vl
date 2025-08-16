cmake_minimum_required(VERSION 3.14)
project(common LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(common SHARED 
  src/common.cpp)

target_include_directories(common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
)
install(TARGETS common
    EXPORT wust_vlTargets
    LIBRARY DESTINATION lib/wust_vl
    ARCHIVE DESTINATION lib/wust_vl
    RUNTIME DESTINATION bin/wust_vl
)

install(DIRECTORY include/ DESTINATION include/wust_vl)