cmake_minimum_required(VERSION 3.5)
project(alignment)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})
link_directories(${realsense2_LIBRARY_DIRS})
add_definitions(${realsense2_DEFINITIONS})


find_package(OpenCV  REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
add_definitions(${OPENCV_DEFINITIONS})


set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(alignment  align.cpp )
target_link_libraries(alignment ${DEPENDENCIES})
