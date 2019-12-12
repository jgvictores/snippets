cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ratethread main.cpp
                          RateThread.hpp
                          RateThread.cpp)

target_link_libraries(ratethread pthread)
