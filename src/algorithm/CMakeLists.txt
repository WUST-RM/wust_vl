add_library(wust_vl_algorithm SHARED)
target_sources(
    wust_vl_algorithm
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/pnp_solver.cpp
)
target_include_directories(wust_vl_algorithm PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/wust_vl>
)
target_link_libraries(wust_vl_algorithm 
    ${OpenCV_LIBS}
    Eigen3::Eigen
)