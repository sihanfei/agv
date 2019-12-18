cmake_minimum_required(VERSION 2.8.3)
project(common)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
  # LIBRARIES glog
)


include_directories(
  include
  ~/work/superg_agv/src/third_party/glog/include
  ~/superg_agv/src/third_party/glog/include

  ~/superg_agv/src/common/include  
  ~/work/superg_agv/src/common/include 
  ${catkin_INCLUDE_DIRS}

)
message("======================== catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")

# set(project_INCLUDE_DIRS
#   ~/superg_agv/src/third_party/higo_adcu/include
#   ~/work/superg_agv/src/third_party/new_higo_adcu/include
# )


link_directories(
  ~/work/superg_agv/src/third_party/glog/lib
  ~/superg_agv/src/third_party/glog/lib
)


add_library(glog_helper
  src/glog_helper.cpp
)
add_dependencies(glog_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(glog_helper
  glog
  ${catkin_LIBRARIES}
)


install(TARGETS glog_helper
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
