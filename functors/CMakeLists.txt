
# Point to line functor
add_library(point_to_line_functor SHARED point_to_line_functor.cpp)
target_link_libraries(point_to_line_functor
    point
    line
)

# Point to plane functor
add_library(point_to_plane_functor SHARED point_to_plane_functor.cpp)
target_link_libraries(point_to_plane_functor
    point
    plane
)

# 2D point distance function
add_library(distcostfunction SHARED distcostfunction.cpp)
target_link_libraries(distcostfunction
    point2d
)

# Polynomial cost function
add_library(polynomialcostfuntion SHARED polynomialcostfuntion.cpp)
target_link_libraries(polynomialcostfuntion
    point2d
    polynomial
)

