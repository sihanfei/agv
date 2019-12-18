cmake_minimum_required(VERSION 2.8.3)
project(perception)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  perception_msgs
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  perception_sensor_msgs
  )

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES perception
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  #LIBRARIES glog_helper
)

include_directories(
  include
  ~/work/superg_agv/src/third_party/glog/include
  ~/work/superg_agv/src/common/include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ~/work/superg_agv/src/third_party/glog/lib
  /usr/local/lib
  ${OpenCV_LIBRARIES}
)

add_executable(per_sender_pub
  src/per_sender_pub.cpp
)
add_dependencies(per_sender_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_sender_pub
  ${catkin_LIBRARIES}
)


add_executable(per_obs
  src/obstacle_handing.cpp
)
add_dependencies(per_obs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_obs
  ${catkin_LIBRARIES}
  glog_helper
)

add_executable(roi_obs
  src/roi_detection_handing.cpp
)
add_dependencies(roi_obs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roi_obs
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  glog_helper
)

add_executable(per_test
  src/obstacle_handing_test.cpp  
)
add_dependencies(per_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(per_test
  ${catkin_LIBRARIES}
  glog_helper
)


set(loc_test_SRCS
  src/loc_test.cpp
)

add_executable(loc_test  ${loc_test_SRCS})

target_link_libraries(loc_test ${catkin_LIBRARIES})

add_dependencies(loc_test 
    ${catkin_EXPORTED_TARGETS}
    ${PCL_LIBRARIES}
    )