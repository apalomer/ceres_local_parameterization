

# Exception
add_library(exception SHARED exception.cpp)

# Functions library
add_library(functions SHARED functions.cpp)

# Point
add_library(point SHARED point.cpp)
if(${PCL_FOUND})
    target_link_libraries(point
        ${PCL_LIBRARIES}
    )
endif()

# Point2d
add_library(point2d SHARED point2d.cpp)

# Line
add_library(line SHARED line.cpp)

# Line
add_library(plane SHARED plane.cpp)

# Polynomial
add_library(polynomial SHARED polynomial.cpp)
