cmake_minimum_required(VERSION 2.8.3)
project(map)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  hmi_msgs
  tf
)

##添加此package以便使用矩阵运算
# find_package(Eigen3 REQUIRED) 

# PKG_CHECK_MODULES(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIRS
        NAMES yaml_cpp.h
        PATHS ${YAML_CPP_INCLUDE_DIRS})
# message("=========================================YAML_CPP_INCLUDE_DIRS = ${YAML_CPP_INCLUDE_DIRS}")
find_library(YAML_CPP_LIBRARY
        NAMES YAML_CPP
        PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})
# message("=========================================YAML_CPP_LIBRARIES = ${YAML_CPP_LIBRARIES}")

catkin_package(
  CATKIN_DEPENDS
  roscpp 
  LIBRARIES tcp_process
  LIBRARIES glog_helper
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}

  ~/superg_agv/src/drivers/udp_socket/include  
  ~/work/superg_agv/src/drivers/udp_socket/include 

  ~/work/superg_agv/src/third_party/glog/include
  ~/superg_agv/src/third_party/glog/include

  ~/superg_agv/src/common/include  
  ~/work/superg_agv/src/common/include 

  ~/superg_agv/src/third_party/spdlog-1.x/include
  ~/work/superg_agv/src/third_party/spdlog-1.x/include
)

set(project_SRCS
  src/ref_sender.cpp
  src/coordinate.cpp
  )

set(project_INCLUDE_DIRS
  include
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(ref_sender  ${project_SRCS} ${project_INCLUDE_DIRS})
target_link_libraries(ref_sender ${catkin_LIBRARIES})
add_dependencies(ref_sender ${catkin_EXPORTED_TARGETS})


add_executable(planning_sender_pub
  src/planning_sender_pub.cpp
)
add_dependencies(planning_sender_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_sender_pub   ${catkin_LIBRARIES} yaml-cpp)


set(ref_get_SRCS
  src/ref_get_main.cpp
)

add_executable(ref_get  ${ref_get_SRCS})

target_link_libraries(ref_get ${catkin_LIBRARIES})

add_dependencies(ref_get ${catkin_EXPORTED_TARGETS})
