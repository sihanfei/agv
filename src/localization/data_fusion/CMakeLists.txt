cmake_minimum_required(VERSION 2.8.3)
project(data_fusion)

## 加了cmake_modules以便使用矩阵运算
find_package(catkin REQUIRED COMPONENTS geometry_msgs location_msgs roscpp std_msgs location_sensor_msgs hmi_msgs cmake_modules dynamic_reconfigure)
add_compile_options(-std=c++11)
##添加此package以便使用矩阵运算
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/IMU_global_conf.cfg
  cfg/IMU_UWB_conf.cfg 
  cfg/IMU_Lidar_conf.cfg 
  cfg/IMU_ZUPT_UWB_conf.cfg 
  cfg/IMU_ZUPT_Lidar_conf.cfg 
  cfg/IMU_Fixed_Lidar_conf.cfg 
  cfg/IMU_ZUPT_Fixed_Lidar_conf.cfg 
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs location_msgs roscpp std_msgs location_sensor_msgs hmi_msgs
)

set(project_INCLUDE_DIRS
  ~/work/superg_agv/src/localization/include   ##请大家添加自己的工作区目录，不要删除已有
)

set(project_SRCS
  src/FusionUWB.cpp
  src/FusionFixedLidar.cpp
  src/FusionLidar.cpp
  src/FusionZUPTUWB.cpp
  src/FusionZUPTFixedLidar.cpp
  src/FusionZUPTLidar.cpp
  src/CoordinateSystem.cpp
  src/FusionDR.cpp
  src/FusionCenter.cpp
  src/MatchForTime.cpp
  src/CalibrateForIMU.cpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
	${project_INCLUDE_DIRS}
)

##添加此处以便使用矩阵运算
include_directories(${Eigen3_INCLUDE_DIRS}) 

##添加此处以便使用矩阵运算
add_definitions(${EIGEN_DEFINITIONS}) 

add_executable(FusionCenter
  ${project_SRCS}
)
add_dependencies(FusionCenter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FusionCenter
  ${catkin_LIBRARIES}
)

