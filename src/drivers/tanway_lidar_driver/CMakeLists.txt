cmake_minimum_required(VERSION 2.8.3)
project(tanway_lidar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
sensor_msgs
  pcl_ros 
  pcl_conversions
  udp_socket
  common
)

find_package(PCL REQUIRED)

PKG_CHECK_MODULES(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIRS
        NAMES yaml_cpp.h
        PATHS ${YAML_CPP_INCLUDE_DIRS})
message("=========================================YAML_CPP_INCLUDE_DIRS = ${YAML_CPP_INCLUDE_DIRS}")
find_library(YAML_CPP_LIBRARY
        NAMES YAML_CPP
        PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})
message("=========================================YAML_CPP_LIBRARIES = ${YAML_CPP_LIBRARIES}")

catkin_package(
  CATKIN_DEPENDS
  roscpp 
  pcl_ros 
  common
  LIBRARIES udp_process
  LIBRARIES glog_helper
  # dynamic_reconfigure
#  INCLUDE_DIRS include
#  LIBRARIES monitor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ~/work/superg_agv/src/third_party/glog/include
  ~/superg_agv/src/third_party/glog/include

  ~/superg_agv/src/drivers/udp_socket/include  
  ~/work/superg_agv/src/drivers/udp_socket/include 

  ~/superg_agv/src/common/include  
  ~/work/superg_agv/src/common/include 

  ~/superg_agv/src/third_party/spdlog-1.x/include
  ~/work/superg_agv/src/third_party/spdlog-1.x/include
)
link_directories(
  ~/work/superg_agv/src/third_party/glog/lib
  ~/superg_agv/src/third_party/glog/lib
)
add_executable(tanway_m16_lidar_node
  src/tanway_m16_lidar_node.cpp
  src/TW_m16_color.cpp
)
add_dependencies(tanway_m16_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tanway_m16_lidar_node
   glog_helper
   udp_process
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_executable(tanway_m16_syncLidars 
  src/tanway_m16_syncLidars.cpp
)
add_dependencies(tanway_m16_syncLidars ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tanway_m16_syncLidars
  ${catkin_LIBRARIES}
)