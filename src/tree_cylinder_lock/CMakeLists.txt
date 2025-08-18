cmake_minimum_required(VERSION 3.0.2)
project(tree_cylinder_lock)

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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(cylinder_pcd_node src/cylinder_pcd_node.cpp)
target_link_libraries(cylinder_pcd_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(forest_cylinder_node src/forest_cylinder_node.cpp)
target_link_libraries(forest_cylinder_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
