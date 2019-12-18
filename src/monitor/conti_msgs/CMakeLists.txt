cmake_minimum_required(VERSION 2.8.3)
project(conti_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  std_msgs
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_message_files(FILES
  ContiArsAbcData.msg
  ContiArs60cData.msg
  ContiArs60bData.msg
  ContiPacket.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS 
  message_runtime 
  std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
