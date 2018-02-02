
# Line local parameterization
 add_custom_target(line_parameterization SOURCES line_parameterization.h)
target_link_libraries(line_parameterization
    glog
    ceres
)

# Plane local parameterization
add_library(plane_parameterization SHARED plane_parameterization.cpp)
target_link_libraries(plane_parameterization
    glog
    ceres
)

# Plane local parameterization
add_library(polynomial_parameterization SHARED polynomialparameterization.cpp)
target_link_libraries(polynomial_parameterization
    glog
    ceres
)
