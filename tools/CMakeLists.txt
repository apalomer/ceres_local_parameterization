

# Exception
add_library(exception SHARED exception.cpp)

# Functions library
add_library(functions SHARED functions.cpp)

# Point
add_custom_target(point SOURCES point.h)
target_link_libraries(point
    exception
)
if(${PCL_FOUND})
    target_link_libraries(point
        ${PCL_LIBRARIES}
    )
endif()

# Point2d
add_custom_target(point2d SOURCES point2d.h)

# Line
add_custom_target(line SOURCES line.h)
target_link_libraries(line
    exception
)
if(${PCL_FOUND})
    target_link_libraries(line
        ${PCL_LIBRARIES}
    )
endif()

# Line
add_custom_target(plane SOURCES plane.h)
target_link_libraries(plane
    exception
)

# Polynomial
add_custom_target(polynomial SOURCES polynomial.h)
