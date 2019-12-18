cmake_minimum_required(VERSION 2.8.3)
project(power_control_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
   message_generation
   roscpp
   rospy
   std_msgs
)


add_message_files(
   FILES
   PowerControlCmd.msg
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  
 )


catkin_package(
  CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)


include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)


