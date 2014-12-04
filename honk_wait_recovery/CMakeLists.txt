cmake_minimum_required(VERSION 2.8.3)
project(honk_wait_recovery)

find_package(catkin REQUIRED COMPONENTS costmap_2d
                                        nav_core
                                        pluginlib
                                        roscpp
                                        std_msgs
                                        tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(INCLUDE_DIRS include
               LIBRARIES honk_wait_recovery
               CATKIN_DEPENDS costmap_2d
                              nav_core
                              pluginlib
                              roscpp
                              std_msgs
                              tf
               DEPENDS system_lib)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(honk_wait_recovery src/honk_wait_recovery.cpp)

target_link_libraries(honk_wait_recovery ${catkin_LIBRARIES})

install(TARGETS honk_wait_recovery
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
