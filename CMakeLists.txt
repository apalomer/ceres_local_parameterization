#####################################################################################
#                                                                                   #
#                      CMake Configuration                                          #
#                                                                                   #
#####################################################################################

# Minimum version required
cmake_minimum_required(VERSION 2.8)

# Project Name
project(ceres_local_parameterization)

# Set bin output
set(CMAKE_BIN "${CMAKE_SOURCE_DIR}/bin")
set(CMAKE_LIB "${CMAKE_SOURCE_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BIN})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BIN})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIB})

# Add root directory to include
include_directories(${CMAKE_SOURCE_DIR})

# c++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#####################################################################################
#                                                                                   #
#                      Dependencies                                                 #
#                                                                                   #
#####################################################################################

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# PCL
find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
if(${PCL_FOUND})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCOMPILE_PCL")
endif()

#####################################################################################
#                                                                                   #
#                      Code                                                         #
#                                                                                   #
#####################################################################################

# Add code
add_subdirectory(tools)
add_subdirectory(functors)
add_subdirectory(local_parameterizations)
add_subdirectory(iterator_viewers)
add_subdirectory(apps)
