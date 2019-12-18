#ifndef TP_CONTROL_H_
#define TP_CONTROL_H_

#include "control.h"
#include "control_utils.h"
#include "pid_control.h"
#include "ros/ros.h"

#include <iostream>

#include "control_msgs/ADControlAGV.h"
#include "control_msgs/com2veh.h"
#include "location_msgs/FusionDataInfo.h"
#include <common_msgs/PathPoint.h>
#include <plan_msgs/DecisionInfo.h>
#include <control_msgs/AGVStatus.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <location_sensor_msgs/IMUAndGNSSInfo.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>



using namespace std;

namespace control
{
class TPControl
{
public:

  TPControl(ros::NodeHandle &nh);
  ~TPControl();

  //主循环函数
  void tpControl();

  //初始化函数
  void initialization(ros::NodeHandle &nh);

  //回调函数
  void recvDecisionInfoCallback(const plan_msgs::DecisionInfo &msg);
  void recvAGVStatusCallback(const control_msgs::AGVStatus &msg);
  void recvPrescanDecisionInfoCallback(const geometry_msgs::Pose &msg);
  void recvPrescanAGVStatusCallback(const nav_msgs::Odometry &msg);
protected:

  int mode;
  int path_mode;
  int decision_start_flag;
  int agvstatus_start_flag;
  int tinypark_success_flag;
  int dir_judge_flag;
  int init_dir;

  double tp_speed;

  double dx;
  double dy;
  double dheading;

  double park_err_tolerance;

  control_msgs::ADControlAGV control_msg;
  geometry_msgs::PoseStamped prescan_control_msg;
  control_msgs::AGVStatus agv_status;
  control_msgs::AGVStatus prescan_agv_status;

  ros::Subscriber decision_info_sub;
  ros::Subscriber agv_status_sub;
  ros::Publisher control_vcu_pub;
};
}

#endif
