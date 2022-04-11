cmake_minimum_required(VERSION 2.8.12)
project(points2octomap)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        tf
        sensor_msgs
        nav_msgs
        )

find_package(PCL REQUIRED)


catkin_package(
        CATKIN_DEPENDS
        roscpp
        pcl_ros
        pcl_conversions
        tf
        sensor_msgs
        nav_msgs
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(points2octomap src/points2octomap.cpp)
target_link_libraries(points2octomap ${catkin_LIBRARIES})
add_dependencies(points2octomap
        ${catkin_EXPORTED_TARGETS})

