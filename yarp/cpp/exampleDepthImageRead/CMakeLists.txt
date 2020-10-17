cmake_minimum_required(VERSION 3.12)

project(exampleDepthImageRead)

find_package(YARP 3.3 REQUIRED COMPONENTS os sig)

add_executable(exampleDepthImageRead exampleDepthImageRead.cpp)

target_link_libraries(exampleDepthImageRead YARP::YARP_os
                                            YARP::YARP_init
                                            YARP::YARP_sig)
