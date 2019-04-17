#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.13)

project(RealSensePCL)

# Save the command line compile commands in the build output
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package( realsense2 REQUIRED)
include_directories(${realsense2_INCLUDE_DIRS})
link_directories(${realsense2_LIBRARY_DIRS})
add_definitions(${realsense2_DEFINITIONS})

find_package(OpenGL REQUIRED)


find_package(OpenGL)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${OPENGL_LIBRARY_DIR})
add_definitions(${OPENGL_DEFINITIONS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
link_directories(${GLUT_LIBRARY_DIR})
add_definitions(${GLUT_DEFINITIONS})

find_package( glfw3 REQUIRED)
include_directories(${GLFW_INCLUDE_DIRS})
link_directories(${GLFW_LIBRARY_DIR})    # libglfw.so.3.1

set(DEPENDENCIES realsense2 glfw ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} )
add_executable(rs-pcl example.hpp stb_easy_font.h rs-pcl.cpp rsconversion.h view_helper.h)
target_link_libraries(rs-pcl  ${DEPENDENCIES} librealsense2.so.2.19.2)


