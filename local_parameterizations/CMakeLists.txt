
# Line local parameterization
add_library(line_parameterization SHARED line_parameterization.cpp)
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

