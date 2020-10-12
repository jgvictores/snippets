cmake_minimum_required(VERSION 3.8)

project(bulletExample LANGUAGES CXX)

find_package(Bullet REQUIRED)

include_directories(${BULLET_INCLUDE_DIR})
add_executable(main main.cpp)
target_link_libraries(main ${BULLET_LIBRARIES})
