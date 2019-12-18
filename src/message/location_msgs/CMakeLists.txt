cmake_minimum_required(VERSION 2.8.3)
project(location_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  common_msgs
  geometry_msgs
)


add_message_files(
  FILES
  FusionDataInfo.msg
)

#generate_messages(DEPENDENCIES ${MSG_DEPS})

generate_messages(DEPENDENCIES
  std_msgs
  common_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)


