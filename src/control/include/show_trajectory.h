#ifndef SHOW_TRAJECTORY_H_
#define SHOW_TRAJECTORY_H_

#include "ros/ros.h"

#include "iostream"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include "vector"
#include "math.h"

namespace display
{

#define DATA_PATH "/work/superg_agv/src/data/data_test"
#define DATA_NAME "/route_data.bin"
#define DATA_NAME_SIM_ROUTE "/sim_route_data.bin"
#define DATA_NAME_SIM_RING "/sim_ring_data.bin"
#define DATA_NAME_REAL_ROUTE "/real_route_data.bin"
#define DATA_NAME_REAL_RING "/real_ring_data.bin"

#define pi 3.1415926

}

#endif
