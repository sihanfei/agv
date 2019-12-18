cmake_minimum_required(VERSION 2.8.3)
project(simlink_udp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib#
  perception_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES udp_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(simlink_udp_node
  src/simlink_udp_node.cpp
  src/udp_process.cpp
)
add_dependencies(simlink_udp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simlink_udp_node
  ${catkin_LIBRARIES}
  yaml-cpp
)