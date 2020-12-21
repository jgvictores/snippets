cmake_minimum_required(VERSION 3.12)

project(exampleBullet LANGUAGES CXX)

find_package(Bullet REQUIRED)

add_executable(exampleBullet main.cpp b3RobotSimulatorClientAPI.cpp)

include_directories(${BULLET_INCLUDE_DIRS})

#target_link_libraries(myexe ${BULLET_LIBRARIES})
target_link_libraries(exampleBullet
                      BulletExampleBrowserLib
                      BulletSoftBody
                      BulletInverseDynamicsUtils
                      BulletInverseDynamics
                      OpenGLWindow
                      BussIK
                      BulletDynamics
                      BulletCollision
                      LinearMath
                      Bullet3Common
                      BulletRobotics
                      BulletRoboticsGUI)
