cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_object_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  darknet_ros_msgs
  object_detector_msgs
  message_filters
)

find_package(PCL 1.10 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/point_cloud_object_detector.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(point_cloud_object_detector_node src/point_cloud_object_detector_node.cpp src/point_cloud_object_detector.cpp)
add_dependencies(point_cloud_object_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_object_detector_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_subscriber src/pcl_subscriber.cpp)
target_link_libraries(pcl_subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bboxes_subscriber src/bboxes_subscriber.cpp)
target_link_libraries(bboxes_subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_message_filters src/test_message_filters.cpp)
target_link_libraries(test_message_filters ${catkin_LIBRARIES} ${PCL_LIBRARIES})