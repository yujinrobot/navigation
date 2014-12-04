cmake_minimum_required(VERSION 2.8.3)
project(clear_rotate_recovery)

find_package(catkin REQUIRED COMPONENTS angles
                                        base_local_planner
                                        costmap_2d
                                        geometry_msgs
                                        nav_core
                                        pluginlib
                                        roscpp
                                        std_msgs
                                        tf)

catkin_package(INCLUDE_DIRS include
               LIBRARIES clear_rotate_recovery
               CATKIN_DEPENDS angles
                              base_local_planner
                              costmap_2d
                              geometry_msgs
                              nav_core
                              pluginlib
                              roscpp
                              std_msgs
                              tf
               DEPENDS system_lib)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(clear_rotate_recovery src/clear_rotate_recovery.cpp)

target_link_libraries(clear_rotate_recovery ${catkin_LIBRARIES})

install(TARGETS clear_rotate_recovery
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
