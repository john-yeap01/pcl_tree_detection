cmake_minimum_required(VERSION 3.0.2)
project(trunk_detection)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-std=c++14 -O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# PCL (we’ll use filters/segmentation/features later)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
                 pcl_ros pcl_conversions tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud_accumulator src/cloud_accumulator.cpp)
target_link_libraries(cloud_accumulator ${catkin_LIBRARIES})
