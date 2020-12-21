cmake_minimum_required(VERSION 3.12)

project(exampleBullet LANGUAGES CXX)

find_package(Bullet REQUIRED)

add_executable(exampleBullet main.cpp b3RobotSimulatorClientAPI.cpp)

include_directories(${BULLET_INCLUDE_DIRS})

#message(${BULLET_LIBRARIES}) # BulletDynamics BulletCollision LinearMath BulletSoftBody

target_link_libraries(exampleBullet
                      ${BULLET_LIBRARIES}
                      BulletExampleBrowserLib
                      BulletInverseDynamicsUtils
                      BulletInverseDynamics
                      OpenGLWindow
                      BussIK
                      Bullet3Common
                      BulletRobotics
                      BulletRoboticsGUI)
