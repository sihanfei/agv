cmake_minimum_required(VERSION 2.8.3)
project(control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  novalet_gps
  novatel_gps_msgs
  tf
)

catkin_package()

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(pid_control
  src/functions/pid_control.cpp
)

add_library(rtk_control
  src/functions/rtk_control.cpp
)
add_library(lf_control
  src/functions/lf_control.cpp
)
add_library(gp_control
  src/functions/gp_control.cpp
)
add_library(tp_control
  src/functions/tp_control.cpp
)
add_library(dg_control
  src/functions/dg_control.cpp
)

add_library(save_route_point
  src/functions/save_route_point.cpp
)

add_executable(control 
  src/main/control.cpp
)
add_executable(load_control 
  src/main/load_control.cpp
)
add_executable(showtrajectory src/main/show_trajectory.cpp src/functions/STrajectory.cpp)


add_dependencies(pid_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(save_route_point 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(rtk_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(lf_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(gp_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(tp_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(dg_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(load_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pid_control
  ${catkin_LIBRARIES}
)

target_link_libraries(save_route_point
  ${catkin_LIBRARIES}
)

target_link_libraries(rtk_control 
  rt
  pid_control
  ${catkin_LIBRARIES}
)

target_link_libraries(control 
  lf_control
  gp_control
  rtk_control
  tp_control
  dg_control
  save_route_point 
  ${catkin_LIBRARIES}
)

target_link_libraries(load_control 
  tp_control
  ${catkin_LIBRARIES}
)

target_link_libraries(showtrajectory ${catkin_LIBRARIES})

target_link_libraries(lf_control 
  rt
  pid_control
  ${catkin_LIBRARIES}
)

target_link_libraries(gp_control 
  rt
  pid_control
  ${catkin_LIBRARIES}
)

target_link_libraries(tp_control 
  rt
  pid_control
  ${catkin_LIBRARIES}
)

target_link_libraries(dg_control 
  rt
  pid_control
  ${catkin_LIBRARIES}
)

