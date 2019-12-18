cmake_minimum_required(VERSION 2.8.3)
project(decision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 std_msgs
 message_generation
 common_msgs
 location_msgs
 perception_msgs
 plan_msgs
 control_msgs
 map_msgs
 status_msgs
 dynamic_reconfigure
)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Generate dynamic_reconfigure server
# generate_dynamic_reconfigure_options(
# cfg/TrajectotyParameterINFO_1.cfg
# cfg/TrajectotyParameterINFO_2.cfg
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)



add_library(decision_pkg
 src/common/cartesian_point.cpp
 src/common/frenet_point.cpp
 src/common/cartesian_frenet_converter.cpp
 src/common/control.cpp
 src/common/perception_Info.cpp
 src/common/ref_line.cpp
 src/common/vcu.cpp
 src/common/vms_cmd.cpp
 src/common/speed_interval.cpp
 src/common/box2d.cpp
 src/lattice/trajectory.cpp
 src/lattice/trajectory_curve.cpp
 src/lattice/lattice_planner.cpp
 src/lattice/trajectory_pair.cpp
 src/curve/quartic_polynomial.cpp
 src/curve/quintic_polynomial.cpp
 src/decision/decision.cpp
 src/decision/lane_change.cpp
)

add_dependencies(decision_pkg ${decision_pkg_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(decision_pkg
 ${catkin_LIBRARIES}
)



add_executable(decision src/pnc_node.cpp)

add_dependencies(decision ${decision_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(decision
 decision_pkg
 ${catkin_LIBRARIES}
)
