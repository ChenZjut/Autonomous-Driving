cmake_minimum_required(VERSION 2.8.3)
project(waypoint_follower)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
        tf
        pcl_ros
        roscpp
        std_msgs
        geometry_msgs
        smartcar_msgs
        nav_msgs
        can_msgs
        utility
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        smartcar_msgs
        nav_msgs
        pcl_ros
        can_msgs
        utility
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(pure_pursuit_node
        nodes/pure_persuit/pure_pursuit_node.cpp
        nodes/pure_persuit/pure_pursuit_core.cpp
        nodes/pure_persuit/pure_pursuit_viz.cpp
        )
target_link_libraries(pure_pursuit_node ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_node      ${catkin_EXPORTED_TARGETS})

