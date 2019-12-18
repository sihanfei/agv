cmake_minimum_required(VERSION 2.8.3)
project(perception_camera)

## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs common_msgs perception_sensor_msgs perception_msgs message_filters location_msgs message_generation)

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

if (OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_message_files(
  FILES 
  CameraObstacle.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
 CATKIN_DEPENDS message_runtime ${MSG_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
  ${PCL_INCLUDE_DIRS}
  ${OpenMP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

add_library(camera_base_filter
  src/filter/base_filter.cpp
)
add_dependencies(camera_base_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_base_filter
  ${catkin_LIBRARIES}
)

add_library(camera_normal_kalman_filter
  src/filter/normal_kalman_filter.cpp
)
add_dependencies(camera_normal_kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_normal_kalman_filter
  ${catkin_LIBRARIES}
)

add_library(camera_base_object
  src/sensor_object/base_object.cpp
)
add_dependencies(camera_base_object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_base_object
  ${catkin_LIBRARIES}
  camera_base_filter
  camera_calculate_similarity
)

add_library(camera_camera_object
  src/sensor_object/camera_object.cpp
)
add_dependencies(camera_camera_object ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_camera_object
  ${catkin_LIBRARIES}
  camera_calculate_velocity
)

add_library(camera_base_association
  src/associate/base_association.cpp
)
add_dependencies(camera_base_association ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_base_association
  ${catkin_LIBRARIES}
  camera_base_object
)

add_library(camera_max_association
  src/associate/max_association.cpp
)
add_dependencies(camera_max_association ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_max_association
  ${catkin_LIBRARIES}
  camera_base_object
  camera_base_association
)

add_library(camera_hungarian_association
  src/associate/hungarian_association.cpp
)
add_dependencies(camera_hungarian_association ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_hungarian_association
  ${catkin_LIBRARIES}
  camera_base_object
  camera_base_association
)

add_library(camera_tools
  src/utils/tools.cpp
)
add_dependencies(camera_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_tools
  ${catkin_LIBRARIES}
  camera_camera_object
  camera_normal_kalman_filter
)

add_library(camera_calculate_similarity
  src/utils/calculate_similarity.cpp
)
add_dependencies(camera_calculate_similarity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_calculate_similarity
  ${catkin_LIBRARIES}
)

add_library(camera_calculate_velocity
  src/utils/calculate_velocity.cpp
)
add_dependencies(camera_calculate_velocity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_calculate_velocity
  ${catkin_LIBRARIES}
)

add_library(camera_perception_camera_lib
  src/perception_camera.cpp
)
add_dependencies(camera_perception_camera_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_perception_camera_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_tools
)

add_executable(camera_obstacle_detection
  src/main.cpp
)
add_dependencies(camera_obstacle_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_obstacle_detection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_LIBRARIES}
  camera_max_association
  camera_perception_camera_lib
)
