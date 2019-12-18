cmake_minimum_required(VERSION 2.8.3)
project(com2agv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control_msgs
  tf
  status_msgs
  plan_msgs
  
)

catkin_package()

set(project_SRCS
  src/com2agv.cpp
  src/control_algo.cpp
  src/relay.cpp
  src/agv2p2.cpp
)

set(project_INCLUDE_DIRS
  include
  ~/superg_agv/src/third_party/new_higo_adcu/include
  ~/work/superg_agv/src/third_party/new_higo_adcu/include
)

link_directories(
  ~/superg_agv/src/third_party/new_higo_adcu/lib
  ~/work/superg_agv/src/third_party/new_higo_adcu/lib  
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${project_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)


## add_executable(abc_sub
##  ## )
## add_dependencies(abc_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## target_link_libraries(abc_sub
##  ${catkin_LIBRARIES}
## )

add_executable(com2agv
  src/com2agv.cpp
  src/control_algo.cpp
  src/relay.cpp
  src/agv2p2.cpp
)
add_dependencies(com2agv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(com2agv WiseADCUSdk pthread rt ${catkin_LIBRARIES})

add_executable(KeyboardTeleop
  src/KeyboardTeleop.cpp
)
add_dependencies(KeyboardTeleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KeyboardTeleop
  ${catkin_LIBRARIES}
)