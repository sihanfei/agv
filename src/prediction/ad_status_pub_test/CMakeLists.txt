cmake_minimum_required(VERSION 2.8.3)
project(prediction)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  status_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(ad_status_pub_t
  src/ad_status_pub_test.cpp
)
add_dependencies(ad_status_pub_t ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ad_status_pub_t
  ${catkin_LIBRARIES}
)


add_executable(obs_time_test
  src/obs_time_test.cpp
)
add_dependencies(obs_time_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obs_time_test
  ${catkin_LIBRARIES}
)