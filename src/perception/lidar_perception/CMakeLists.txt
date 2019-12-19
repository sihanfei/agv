cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(lidar_perception)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  perception_msgs
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS})

add_executable(lidarDetection
  src/lidarObstacle.cpp
)

add_dependencies(lidarDetection 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lidarDetection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
