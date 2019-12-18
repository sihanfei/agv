cmake_minimum_required(VERSION 2.8.3)
project(rviz_monitor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  visualization_msgs 
  nav_msgs 
  tf 
  dynamic_reconfigure 
  common_msgs 
  plan_msgs 
  map_msgs 
  message_filters 
  pcl_ros 
  pcl_conversions
  udp_socket
  diagnostic_updater
  perception_sensor_msgs
)
find_package(OpenCV)
# find_package(glog)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  rviz_monitor
  common_msgs
  plan_msgs
  map_msgs
  
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/myconfig.cfg
  cfg/myconfig_1.cfg
  cfg/HmiControlAD.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp 
  pcl_ros 
  std_msgs 
  dynamic_reconfigure
  hmi_msgs
  LIBRARIES udp_process
  # LIBRARIES glog_helper
  # dynamic_reconfigure
#  INCLUDE_DIRS include
#  LIBRARIES monitor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ~/work/superg_agv/src/third_party/new_higo_adcu/include
  ~/superg_agv/src/third_party/new_higo_adcu/include

  # ~/work/superg_agv/src/third_party/glog/include
  # ~/superg_agv/src/third_party/glog/include

  ~/superg_agv/src/drivers/udp_socket/include  
  ~/work/superg_agv/src/drivers/udp_socket/include 

  ~/superg_agv/src/common/include  
  ~/work/superg_agv/src/common/include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

  /usr/local/include/yaml-cpp

  ~/work/superg_agv/src/third_party/spdlog-1.x/include

  # ${glog_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)
# message("========================glog INCLUDE_DIRS = ${glog_INCLUDE_DIRS}")

# set(project_INCLUDE_DIRS
#   ~/superg_agv/src/third_party/higo_adcu/include
#   ~/work/superg_agv/src/third_party/new_higo_adcu/include
# )

link_directories(

  ~/work/superg_agv/src/third_party/new_higo_adcu/lib  
  ~/superg_agv/src/third_party/new_higo_adcu/lib  

  # ~/work/superg_agv/src/third_party/glog/lib
  # ~/superg_agv/src/third_party/glog/lib

  /usr/local/lib
)

#test
# if(CATKIN_ENABLE_TESTING)
# find_package(rostest REQUIRED)
# add_rostest_gtest(times_test test/times_test.test test/times_test.cpp)
# target_link_libraries(times_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
# endif()


add_executable(ros_check_node
  src/ros_check_pub.cpp
)
add_dependencies(ros_check_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_check_node
  ${catkin_LIBRARIES}
)

add_executable(IPC_monitor_node
  src/IPC_monitor_node.cpp
)
add_dependencies(IPC_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IPC_monitor_node
  ${catkin_LIBRARIES}
)

add_executable(rviz_visualization_pub
  src/rviz_visualization_pub.cpp
  src/rviz_reference_line.cpp
  src/rviz_obstacle_sim.cpp
  src/rviz_select_point.cpp
  src/rviz_show_map.cpp
  src/get_map_xy_color.cpp
  # src/find_closest_ref_point.cpp
  src/rviz_path_decision.cpp
  src/rviz_lcl_planning.cpp

)
add_dependencies(rviz_visualization_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(rviz_visualization_pub
  ${OpenCV_LIBS}
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_executable(hmi_control_ad_node
  src/hmi_control_ad_node.cpp
)
add_dependencies(hmi_control_ad_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hmi_control_ad_node
  ${catkin_LIBRARIES}
)

add_executable(sim_radar_node
  src/sim_radar_node.cpp
)
add_dependencies(sim_radar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_radar_node
 WiseADCUSdk 
 pthread rt 
  ${catkin_LIBRARIES}
)

add_executable(sim_lidar_node
  src/sim_lidar_node.cpp
)
add_dependencies(sim_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_lidar_node
  ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(sim_rslidar_node
  src/sim_rslidar_node.cpp
)
add_dependencies(sim_rslidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_rslidar_node
  ${catkin_LIBRARIES}
)

add_executable(diagnostic_updater_node
  src/diagnostic_updater_node.cpp
)
add_dependencies(diagnostic_updater_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diagnostic_updater_node
  ${catkin_LIBRARIES}
)

add_executable(node_status_sub
  src/node_status_sub.cpp
)
add_dependencies(node_status_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_status_sub
  ${catkin_LIBRARIES}
)

add_executable(node_monitor_node
  src/node_monitor_node.cpp
)
add_dependencies(node_monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_monitor_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(lidar_pc2_pub
  src/lidar_pc2_sub.cpp
)
add_dependencies(lidar_pc2_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_pc2_pub
  ${catkin_LIBRARIES}
)
