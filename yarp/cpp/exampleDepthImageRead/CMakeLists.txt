cmake_minimum_required(VERSION 2.8.0)

# Start a project.
project(exampleDepthImageRead)

find_package(YARP REQUIRED COMPONENTS os sig)

# Set up our main executables.

add_executable(exampleDepthImageRead exampleDepthImageRead.cpp)

target_link_libraries(exampleDepthImageRead YARP::YARP_OS
                                  YARP::YARP_init
                                  YARP::YARP_sig)

include(GNUInstallDirs)

install(TARGETS exampleDepthImageRead
        DESTINATION ${CMAKE_INSTALL_BINDIR})
