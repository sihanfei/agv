cmake_minimum_required(VERSION 2.8.3)
project(rs_lidar_driver)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  perception_sensor_msgs
  rslidar_msgs
  pcl_ros 
  pcl_conversions
  roslib
)
find_package(PCL REQUIRED)
set(libpcap_LIBRARIES -lpcap)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
  roscpp 
  pcl_ros 
  LIBRARIES udp_process
  # LIBRARIES glog_helper
  # dynamic_reconfigure
#  INCLUDE_DIRS include
#  LIBRARIES monitor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include 
  ${catkin_INCLUDE_DIRS}

  ~/work/superg_agv/src/drivers/udp_socket/include 

  ~/work/superg_agv/src/common/include 

  ${OpenCV_INCLUDE_DIRS}

  /usr/local/include/yaml-cpp

  ~/work/superg_agv/src/third_party/spdlog-1.x/include
  ${PCL_INCLUDE_DIRS}
)

link_directories(

  /usr/local/lib
)


add_executable(rs_lidar_node
  src/rs_lidar_node.cpp
  src/rawdata.cc
  src/ImageSegment_linh.cpp
)
add_dependencies(rs_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rs_lidar_node
  ${catkin_LIBRARIES}
  udp_process
  ${PCL_LIBRARIES}
  ${libpcap_LIBRARIES}
)

add_executable(test_pub
  src/test_pub.cpp
)
add_dependencies(test_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pub
  ${catkin_LIBRARIES}
)

add_executable(test_sub
  src/test_sub.cpp
)
add_dependencies(test_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_sub
  ${catkin_LIBRARIES}
)