cmake_minimum_required(VERSION 2.8.3)
project(location_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  std_msgs
)

add_message_files(FILES
  UWBInfo.msg
  IMUAndGNSSInfo.msg
  FixedLidarInfo.msg
  LidarInfo.msg
  DRInfo.msg
)

generate_messages(DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion_data_msgs
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
