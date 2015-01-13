#directory adapted by Juan G Victores 2014 @ UC3M from git/pcl/doc/tutorials/content/sources/iccv2011/src

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(captureToolXYZ)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (captureToolXYZ src/capture_tool.cpp src/openni_capture.cpp)
target_link_libraries (captureToolXYZ ${PCL_LIBRARIES})

