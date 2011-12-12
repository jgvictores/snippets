cmake_minimum_required (VERSION 2.4)
project (cppexamples)

set(CMAKE_BUILD_TYPE Debug)#RelWithDebInfo)
include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost COMPONENTS iostreams)

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

add_executable(orcamerasensor orcamerasensor.cpp)
set_target_properties(orcamerasensor PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(orcamerasensor PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(orcamerasensor ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})
install(TARGETS orcamerasensor DESTINATION . )

