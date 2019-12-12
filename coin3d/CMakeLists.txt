cmake_minimum_required(VERSION 2.8)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/modules)

find_package(SoQt4)
if (NOT SOQT_LIBRARY_FOUND)
  message(STATUS "WARNING: SoQt4 not found, disabling QtCoin GUI plugin. Please install SoQt (http://www.coin3d.org)")
endif ()

add_executable(exampleSoOffscreenRenderer exampleSoOffscreenRenderer.cpp)
target_link_libraries(exampleSoOffscreenRenderer ${SOQT_LIBRARY_DEBUG})

