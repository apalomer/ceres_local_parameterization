
# Point to line functor
add_library(point_to_line_functor SHARED point_to_line_functor.cpp)

# Point to plane functor
add_library(point_to_plane_functor SHARED point_to_plane_functor.cpp)

# 2D point distance function
add_library(distcostfunction SHARED distcostfunction.cpp)

# Polynomial cost function
add_library(polynomialcostfuntion SHARED polynomialcostfuntion.cpp)

