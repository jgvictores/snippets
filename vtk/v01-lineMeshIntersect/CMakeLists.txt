

cmake_minimum_required(VERSION 2.8)
 
PROJECT(PolygonIntersection)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
if (APPLE)
  add_executable(PolygonIntersection MACOSX_BUNDLE main.cpp)
else()
  add_executable(PolygonIntersection main.cpp)
endif()
 
if(VTK_LIBRARIES)
  target_link_libraries(PolygonIntersection ${VTK_LIBRARIES})
else()
  target_link_libraries(PolygonIntersection vtkHybrid )
endif()

