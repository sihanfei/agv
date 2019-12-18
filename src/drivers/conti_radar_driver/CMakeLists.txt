cmake_minimum_required(VERSION 2.8.3)
project(conti_radar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  perception_sensor_msgs
  pcl_ros 
  pcl_conversions
  roslib
  # udp_socket
)
find_package(PCL REQUIRED)

# PKG_CHECK_MODULES(YAML_CPP REQUIRED yaml-cpp)

# find_path(YAML_CPP_INCLUDE_DIRS
#         NAMES yaml_cpp.h
#         PATHS ${YAML_CPP_INCLUDE_DIRS})
# # message("=========================================YAML_CPP_INCLUDE_DIRS = ${YAML_CPP_INCLUDE_DIRS}")
# find_library(YAML_CPP_LIBRARY
#         NAMES YAML_CPP
#         PATHS ${YAML_CPP_LIBRARY_DIRS})

# link_directories(${YAML_CPP_LIBRARY_DIRS})
# # message("=========================================YAML_CPP_LIBRARIES = ${YAML_CPP_LIBRARIES}")


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
  include
  ${catkin_INCLUDE_DIRS}

  # ~/work/superg_agv/src/third_party/glog/include
  # ~/superg_agv/src/third_party/glog/include

  ~/superg_agv/src/drivers/udp_socket/include  
  ~/work/superg_agv/src/drivers/udp_socket/include 

  ~/superg_agv/src/common/include  
  ~/work/superg_agv/src/common/include 

  ~/superg_agv/src/third_party/spdlog-1.x/include
  ~/work/superg_agv/src/third_party/spdlog-1.x/include

  /usr/local/include/yaml-cpp
)

link_directories(
  # ~/work/superg_agv/src/third_party/glog/lib

  # ~/superg_agv/src/third_party/glog/lib
)


add_executable(cannet_radar_node
  src/cannet_radar_node.cpp
)
add_dependencies(cannet_radar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cannet_radar_node
  udp_process
  # glog_helper
  ${catkin_LIBRARIES}
  yaml-cpp
)
