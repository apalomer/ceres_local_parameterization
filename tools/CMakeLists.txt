

# Exception
add_library(exception SHARED exception.cpp)

# Functions library
add_library(functions SHARED functions.cpp)

# Point
add_library(point SHARED point.cpp)
target_link_libraries(point
    exception
)
if(${PCL_FOUND})
    target_link_libraries(point
        ${PCL_LIBRARIES}
    )
endif()

# Point2d
add_library(point2d SHARED point2d.cpp)

# Line
add_library(line SHARED line.cpp)
target_link_libraries(line
    exception
)
if(${PCL_FOUND})
    target_link_libraries(line
        ${PCL_LIBRARIES}
    )
endif()

# Line
add_library(plane SHARED plane.cpp)
target_link_libraries(plane
    exception
)

# Polynomial
add_library(polynomial SHARED polynomial.cpp)
