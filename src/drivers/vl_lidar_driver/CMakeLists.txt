cmake_minimum_required(VERSION 2.8.3)
project(vl_lidar_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  perception_sensor_msgs
  velodyne_msgs
  pcl_ros 
  pcl_conversions
  roslib
)
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS
        NAMES yaml_cpp.h
        PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
        NAMES YAML_CPP
        PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


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

  ~/work/superg_agv/src/drivers/udp_socket/include 

  ~/work/superg_agv/src/common/include 

  ${OpenCV_INCLUDE_DIRS}

  /usr/local/include/yaml-cpp

  ~/work/superg_agv/src/third_party/spdlog-1.x/include
  ${PCL_INCLUDE_DIRS}
)
link_directories(

  /usr/local/lib
)

add_executable(vl_lidar_node
  src/vl_lidar_node.cpp
  src/calibration.cc
  src/pointcloudXYZIR.cc
  src/rawdata.cc
  src/ImageSegment_linh.cpp
)
add_dependencies(vl_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vl_lidar_node
  udp_process
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
  ${YAML_CPP_LIBRARIES}
)


