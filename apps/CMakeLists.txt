
# Plane parameterization
add_executable(fit_plane fit_plane.cpp)
target_link_libraries(fit_plane
    glog
    ceres
    exception
    functions
    point
    plane
    point_to_plane_functor
    plane_parameterization
)

# Line parameterization 
add_executable(fit_line fit_line.cpp)
target_link_libraries(fit_line
    glog
    ceres
    ${PCL_LIBRARIES}
    exception
    functions
    point
    line
    point_to_line_functor
    line_parameterization
)
if(${PCL_FOUND})
    target_link_libraries(fit_line
        line_fit_iterator_viewer
    )
endif()
