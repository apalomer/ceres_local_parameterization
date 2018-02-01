

# Line fit iterator viewer
if(${PCL_FOUND})
    add_library(line_fit_iterator_viewer SHARED line_fit_iterator_viewer.cpp)
    target_link_libraries(line_fit_iterator_viewer
        glog
        ceres
        ${PCL_LIBRARIES}
    )
endif()
