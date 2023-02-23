cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_object_detector)

# add_compile_options(-std=c++11 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  darknet_ros_msgs
  geometry_msgs
  object_detector_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

# PCL
find_package(PCL 1.10 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# point_cloud_object_detector_node
add_executable(point_cloud_object_detector_node
  src/point_cloud_object_detector_node.cpp
  src/point_cloud_object_detector.cpp
)
target_link_libraries(point_cloud_object_detector_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)