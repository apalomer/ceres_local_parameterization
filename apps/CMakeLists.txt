
# Plane parameterization
add_executable(fit_plane fit_plane.cpp)
target_link_libraries(fit_plane
    glog
    ceres
    exception
    functions
    point_to_plane_functor
    plane_parameterization
)

# Line parameterization 
add_executable(fit_line fit_line.cpp)
target_link_libraries(fit_line
    glog
    ceres
    exception
    functions
    point_to_line_functor
)
if(${PCL_FOUND})
    target_link_libraries(fit_line
        ${PCL_LIBRARIES}
        line_fit_iterator_viewer
    )
endif()

# Plane parameterization
add_executable(closest_point_on_polynomial closest_point_on_polynomial.cpp)
target_link_libraries(closest_point_on_polynomial
    glog
    ceres
    distcostfunction
    polynomialcostfuntion
    polynomial_parameterization
)
if(${COMPILE_QT})
    target_link_libraries(closest_point_on_polynomial
        ${QCustomPlot_LIBRARIES}
        ${QT_LIBRARIES}
    )
endif()
