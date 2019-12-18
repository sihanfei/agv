cmake_minimum_required(VERSION 2.8.3)
project(fsm_plan)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hmi_msgs
  status_msgs
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

# set(SENDER_SRCS
#   src/fsm_test.cpp
# )

# add_executable(fsm_test  ${SENDER_SRCS})

# target_link_libraries(fsm_test rt ${catkin_LIBRARIES})

# add_dependencies(fsm_test ${catkin_EXPORTED_TARGETS})


# set(test_SRCS
#   src/test.cpp
# )

# add_executable(test_1  ${test_SRCS})

# target_link_libraries(test_1 ${catkin_LIBRARIES})

# add_dependencies(test_1 ${catkin_EXPORTED_TARGETS})


set(fsm_t_SRCS
  src/fsm_main.cpp
  src/state_base_machine.cpp
)

add_executable(fsm_t  ${fsm_t_SRCS})

target_link_libraries(fsm_t ${catkin_LIBRARIES})

add_dependencies(fsm_t ${catkin_EXPORTED_TARGETS})

SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb") 
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
