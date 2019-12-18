cmake_minimum_required(VERSION 2.8.3)
project(can_wr)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries

FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  control_msgs
  location_sensor_msgs
  perception_sensor_msgs
  perception_msgs
  visualization_msgs
  common_msgs
)

catkin_package()


set(project_SRCS
  src/can_rw_main.cpp
  src/can_rw.cpp
  src/p2.cpp
  src/camera.cpp
  # src/sensor.cpp
  src/can_rw309_main.cpp
  src/can_rw309.cpp
  src/logmanager.cpp
  src/logmanager_node.cpp
#  src/com2veh/src/com2agv_allinfo.cpp
#  src/com2veh/src/control_algo.cpp
)

set(project_INCLUDE_DIRS
  include
  ~/superg_agv/src/third_party/new_higo_adcu/include
  ~/work/superg_agv/src/third_party/new_higo_adcu/include
  ~/superg_agv/src/drivers/can_wr/include
  # ~/work/superg_agv/src/drivers/can_wr/include
  ~/superg_agv/src/third_party/glog/include
  ~/work/superg_agv/src/third_party/glog/include
)

link_directories(
  ~/superg_agv/src/third_party/new_higo_adcu/lib
  ~/work/superg_agv/src/third_party/new_higo_adcu/lib  
  ~/superg_agv/src/third_party/glog/lib
  ~/work/superg_agv/src/third_party/glog/lib
  /usr/local/lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${project_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  /usr/local/include/yaml-cpp
)

#add_executable(can_wr_n  ${project_SRCS})
add_executable(can_wr_n
  src/can_rw_main.cpp
  src/can_rw.cpp
  src/p2.cpp
  src/camera.cpp
  src/ultrasonic.cpp
  src/mobileye.cpp
  src/logdata.cpp
  src/node_status.cpp
  src/driver_monitor.cpp
  src/logmanager.cpp
  src/agv2p2.cpp
  src/relay.cpp
)

target_link_libraries(can_wr_n WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(can_wr_n ${catkin_EXPORTED_TARGETS})

add_executable(can_wr_p2
  src/can_rw_p2.cpp
  src/can_rw.cpp
  src/p2.cpp
  src/camera.cpp
  src/ultrasonic.cpp
  src/mobileye.cpp
  src/logdata.cpp
  src/node_status.cpp
  src/driver_monitor.cpp
  src/logmanager.cpp
  src/relay.cpp
)

target_link_libraries(can_wr_p2 WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(can_wr_p2 ${catkin_EXPORTED_TARGETS})

add_executable(can_wr_camera
  src/can_rw_camera.cpp
  src/can_rw.cpp
  src/p2.cpp
  src/camera.cpp
  src/ultrasonic.cpp
  src/mobileye.cpp
  src/logdata.cpp
  src/node_status.cpp
  src/driver_monitor.cpp
  src/logmanager.cpp
  src/relay.cpp
)

target_link_libraries(can_wr_camera WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(can_wr_camera ${catkin_EXPORTED_TARGETS})

add_executable(can_wr_ultrasonic
  src/can_rw_ultrasonic.cpp
  src/can_rw.cpp
  src/p2.cpp
  src/camera.cpp
  src/ultrasonic.cpp
  src/mobileye.cpp
  src/logdata.cpp
  src/node_status.cpp
  src/driver_monitor.cpp
  src/logmanager.cpp
  src/relay.cpp
)

target_link_libraries(can_wr_ultrasonic WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(can_wr_ultrasonic ${catkin_EXPORTED_TARGETS})

add_executable(can_wr309_n
  src/can_rw309_main.cpp
  src/can_rw309.cpp
)

target_link_libraries(can_wr309_n WiseADCUSdk pthread rt ${catkin_LIBRARIES})

add_dependencies(can_wr309_n ${catkin_EXPORTED_TARGETS})

add_executable(log_manager
  src/logmanager_node.cpp
  src/logmanager.cpp
)

target_link_libraries(log_manager WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES} )

add_dependencies(log_manager ${catkin_EXPORTED_TARGETS})

add_executable(power_control
  src/power_control_node.cpp
  src/power_control.cpp
  src/node_status.cpp
  src/driver_monitor.cpp
  src/logdata.cpp
  src/logmanager.cpp
  # src/can_rw.cpp
  # src/p2.cpp
  # src/camera.cpp
  # src/ultrasonic.cpp
  # src/mobileye.cpp
)

target_link_libraries(power_control WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(power_control ${catkin_EXPORTED_TARGETS})

add_executable(relay_control
  src/relay_control.cpp
  src/relay.cpp
  # src/can_rw.cpp
  # src/p2.cpp
  # src/camera.cpp
  # src/ultrasonic.cpp
  # src/mobileye.cpp
)

target_link_libraries(relay_control WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(relay_control ${catkin_EXPORTED_TARGETS})

# add_executable(testsub
#  src/testsub.cpp
#  src/Apath.cpp
# #  src/k_means.cpp
#  src/k_means_1.cpp
# )

# target_link_libraries(testsub WiseADCUSdk pthread rt -levent ${catkin_LIBRARIES})

# add_dependencies(testsub ${catkin_EXPORTED_TARGETS})

add_executable(ultrasonic_dicesion
  src/ultrasonic_dicesion_node.cpp
  src/ultrasonic_dicesion.cpp
)

target_link_libraries(ultrasonic_dicesion WiseADCUSdk pthread rt yaml-cpp ${catkin_LIBRARIES})

add_dependencies(ultrasonic_dicesion ${catkin_EXPORTED_TARGETS})

add_executable(test_power
  src/testpower.cpp
)

target_link_libraries(test_power WiseADCUSdk pthread rt ${catkin_LIBRARIES})

add_dependencies(test_power ${catkin_EXPORTED_TARGETS})


# add_executable(udp_client
#   src/udp_client.cpp
# )

# target_link_libraries(udp_client WiseADCUSdk pthread rt ${catkin_LIBRARIES})

# add_dependencies(udp_client ${catkin_EXPORTED_TARGETS})

# add_executable(com2agv
#  src/com2veh/src/com2agv_allinfo.cpp
#  src/com2veh/src/control_algo.cpp
# )


#add_dependencies(com2agv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(com2agv WiseADCUSdk pthread rt ${catkin_LIBRARIES})

