cmake_minimum_required(VERSION 2.8.3)
project(gps_imu)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  location_msgs
  location_sensor_msgs
  control_msgs
)
##添加此package以便使用矩阵运算
find_package(Eigen3 REQUIRED) 

catkin_package()


include_directories(
  include ${catkin_INCLUDE_DIRS}
)


set(SENDER_SRCS
  src/prescan_location_sender.cpp
  src/coordinate.cpp
)

add_executable(pl_sender  ${SENDER_SRCS})

target_link_libraries(pl_sender rt ${catkin_LIBRARIES})

add_dependencies(pl_sender ${catkin_EXPORTED_TARGETS})