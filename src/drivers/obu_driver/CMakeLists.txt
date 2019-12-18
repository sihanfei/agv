cmake_minimum_required(VERSION 2.8.3)
project(obu_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

  std_msgs
  sensor_msgs
  common_msgs
  status_msgs
  roscpp
  
)

# find_package(Boost REQUIRED COMPONENTS system)
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
  LIBRARIES tcp_process
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}

~/superg_agv/src/drivers/udp_socket/include  
~/work/superg_agv/src/drivers/udp_socket/include 

/usr/local/include/yaml-cpp

~/superg_agv/src/common/include  
~/work/superg_agv/src/common/include 

  ~/superg_agv/src/third_party/spdlog-1.x/include
  ~/work/superg_agv/src/third_party/spdlog-1.x/include
)


message("===********************************====YAML_CPP_LIBRARIES = ${YAML_CPP_LIBRARIES}")

add_executable(fixed_lidar_info_node

src/fixed_lidar_info_node.cpp
)
add_dependencies(fixed_lidar_info_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fixed_lidar_info_node
  tcp_process
  ${catkin_LIBRARIES}
  yaml-cpp
)




