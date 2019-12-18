cmake_minimum_required(VERSION 2.8.3)
project(cannet_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  
)


catkin_package(
  CATKIN_DEPENDS
  roscpp 
  LIBRARIES udp_process
  LIBRARIES glog_helper
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}

~/superg_agv/src/drivers/udp_socket/include  
~/work/superg_agv/src/drivers/udp_socket/include 

~/work/superg_agv/src/third_party/glog/include
~/superg_agv/src/third_party/glog/include

~/superg_agv/src/common/include  
~/work/superg_agv/src/common/include 

  ~/superg_agv/src/third_party/spdlog-1.x/include
  ~/work/superg_agv/src/third_party/spdlog-1.x/include
)


link_directories(
  ~/work/superg_agv/src/third_party/glog/lib
  ~/superg_agv/src/third_party/glog/lib
  )

add_executable(cannet_node
  src/cannet_node.cpp
)
add_dependencies(cannet_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cannet_node
  udp_process
  glog_helper
  # glog
  ${catkin_LIBRARIES}
)
