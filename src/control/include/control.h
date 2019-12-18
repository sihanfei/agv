#ifndef CONTROL_H_
#define CONTROL_H_

#include "ros/ros.h"
#include "control_utils.h"
#include "pid_control.h"
#include "rtk_control.h"
#include "lf_control.h"
#include "gp_control.h"
#include "tp_control.h"
#include "dg_control.h"
#include "save_route_point.h"

#include <iostream>

//#include "test_msgs/TestType.h"
#include "novalet_gps/NovatelPosition.h"
#include <common_msgs/PathPoint.h>
#include <plan_msgs/DecisionInfo.h>
#include <control_msgs/AGVStatus.h>

#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


namespace control
{

#define DATA_PATH "/work/superg_agv/src/data/data_test"
#define DATA_NAME_SIM_ROUTE "/sim_route_data.bin"
#define DATA_NAME_SIM_RING "/sim_ring_data.bin"
#define DATA_NAME_REAL_ROUTE "/real_route_data.bin"
#define DATA_NAME_REAL_RING "/real_ring_data.bin"

#define PID_CONF_PATH "/work/superg_agv/src/data/data_test"
#define PID_CONF_NAME "/pid_conf.bin"

#define FRE 100
#define BUF_LEN 10


#define R0 6378137.0
#define e 0.0818191908425

}//end namespace control
#endif
