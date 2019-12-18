cmake_minimum_required(VERSION 2.8.3)
project(map_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  common_msgs
)


add_message_files(
  FILES 
  REFPointArray.msg
  )

add_service_files(
  FILES
  QueryClosestLanePointDistance.srv
  QueryLaneIRefInfo.srv
  QueryNextLaneIDArray.srv
)

#generate_messages(DEPENDENCIES ${MSG_DEPS})

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)


