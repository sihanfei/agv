#ifndef LF_CONTROL_H_
#define LF_CONTROL_H_

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
class LFControl
{
public:
  LFControl(ros::NodeHandle &nh);
  ~LFControl();

  ////回调函数
  //仿真回调函数
  void recvPrescanRealtimePosCallback(const nav_msgs::Odometry &msg);
  // AGV1.0 华测组合导航定位信息
  void recvHuacePosCallback(const location_sensor_msgs::IMUAndGNSSInfo &msg);
  //局部路径回调函数
  void recvLocalPathCallback(const plan_msgs::DecisionInfo &msg);
  // AGV1.0 融合定位信息
  void recvFusionPosCallback(const location_msgs::FusionDataInfo &msg);

  void recvReflineStopCallback(const control_msgs::ADControlAGV &msg);

  ////数学运算函数
  double pointDistanceSquare(const positionConf &xy_p, const positionConf &route_point);
  double norm2(const double &dx, const double &dy);

  ////坐标系转换函数
  // Prescan仿真位置传递函数
  void prescanxy2xy(positionConf &xy_p, const positionConf &real_p);
  // 经纬度转换成全局坐标系函数
  void gps2xy(positionConf &xy_p, const positionConf &real_p);
  // 组合导航的全局坐标系转成车辆中心点的全局坐标系函数
  void insgps2center(positionConf &xy_p);
  // 车辆中心点坐标转换成车辆前轴中心坐标函数
  void center2frontaxis_tf(positionConf &xy_p);
  // 全局坐标系转换成车辆坐标系函数
  void xy2vxy(positionConf &targetp_veh, const positionConf &targetp_xy, const positionConf &xy_p);

  ////控制算法函数
  //设置PID参数函数
  void setAnglePID();
  void setSpeedPID();
  //控制模型函数
  double StanleyPreviewLatteralModel(const positionConf &xy_p, const positionConf &vnxy_p,
                                     const positionConf &vnxy_p_fc, const positionConf &vpxy_p);

  //查找奇点函数
  void findStrangePoints();

  //路段截取函数
  void route_cut(const positionConf &rpt);

  //判断前进后退函数
  void forward_backward_judge();

  //查找最近点函数
  int findClosestRefPoint(const positionConf &rpt,const int &previous_loc_num);
  int findClosestFCRefPoint(const positionConf &rpt,const int &ref_loc_num);
  //查找考虑延迟最近点函数
  int findRealMinIndex(const int &min_index_theory_, const positionConf &real_p);
  int findRealMinIndex_ringpath(const int &min_index_theory_, const positionConf &real_p);
  //查找预瞄点函数
  void findDiagPrePoint(positionConf &pre_p, const positionConf &real_p, const int &index, const double &R);
  void findPrePoint(positionConf &pre_p, const positionConf &real_p, const int &index, const double &R);
  void findPrePoint_ringpath(positionConf &pre_p, const positionConf &real_p, const int &index, const double &R);
  //角度角速度限制函数
  void angle_constrain(double &input_value);
  void yawrate_constrain(const double &timenow, const double &timeprevious, double &control_angle,
                         double &control_angle_p);
  //计算曲率半径函数
  double getR(const positionConf &pt1, const positionConf &pt2, const positionConf &pt3);
  //取最小值函数
  double getMinR(const double &R1, const double &R4, const double &R5);
  //计算前后轮转向比函数
  double ComputeSigma(const double &R,const positionConf &_nearst_pos_,const positionConf &_xy_pos_,const positionConf &_pre_pos_,double &_control_angle_f,double &_control_angle_r);
  //查找计算曲率半径的点函数
  void findPoints2ComputeRadius(const int &index, int &index_r1, int &index_f1, int &index_f2, int &index_f3);
  void findPoints2ComputeRadius_ringpath(const int &index, int &index_r1, int &index_f1, int &index_f2, int &index_f3);

  double computespeed(const double &R, const double &straightline_speed);
  double computeheading(const positionConf &_p1_,const positionConf &_p2_);
  double computedheading(const double &_heading1_,const double &_heading2_);
  ////主循环函数
  void lfControl();
  // 3元素平方和
  double sumSquares3D(const double &in_x, const double &in_y, const double &in_z);

protected:
  ////flags
  int start_tip_;
  int16_t math_tip_;
  int plan_flag;
  int path_mode;

  int forward_backward_flag;

  int refline_stop;
  control_msgs::ADControlAGV refline_stop_msg;

  ////structs
  vector<int> strange_points;

  vector<int> route_section_index_;

  vector< positionConf > lattice_route_data_;
  vector< positionConf > route_data_;
  vector< positionConf > previous_route_data_;
  vector< positionConf > route_section_data_;
  positionConf real_position_;
  PIDControl pid_control_angle;
  PIDControl pid_control_speed;
  PIDConf pid_conf_angle;
  PIDConf pid_conf_speed;

  ////temp variables
  double latteral_error;
  double previous_control_angle_f;
  double previous_control_angle_r;
  double time_now;
  double time_previous;
  int indexr1, indexr2, indexr3, indexf1, indexf2, indexf3;
  geometry_msgs::PoseStamped this_pose_stamped;
  nav_msgs::Path path;

  double control_angle_f;
  double control_angle_r;

  int section_decide_num;
  int previous_section_decide_num;

  int previous_closest_num;
  int previous_closest_fc_num;

  ////paramaters from yaml
  // simulation mode or real vehicle mode
  int mode;
  // paramaters for path
  int pathtype;
  int Rmax;
  // paramaters for vehicle
  double turn_ratio;
  double max_turn_angle;
  double center2frontaxis;
  double center2rearaxis;
  // paramaters for INS/GPS installation error
  double insgps_x;
  double insgps_y;
  // paramaters for control model
  int turnleft_ispositive;
  double preview_time;
  double basic_prelength_min;
  double basic_prelength_max;
  double lamda_preview;
  double lamda_stanley;
  double k_stanley;
  double k_soft;
  double yawrate_limit;
  double sys_delay;
  double start2stop_dist;

  double lateral_kp;
  double lateral_ki;
  double lateral_kd;

  double straight_speed;
  double speed_limit;
  double tp_speed;

  double equal_length;

  ////paramaters for gps2xy
  double L0;
  double lamda0;
  double hb;

  ////Subscriber and Publisher
  ros::NodeHandle nh_;
  // ros::Subscriber bestpos_sub_;
  // ros::Subscriber Inspva_sub_;
  // ros::Subscriber velocity_sub_;
  // ros::Subscriber route_sub_;
  // ros::Subscriber prescan_position_sub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber local_path_sub_;
  ros::Subscriber refline_stop_msg_sub_;

  ros::Publisher ref_point_pub_;
  ros::Publisher pre_point_pub_;
  ros::Publisher veh_point_pub_;
  ros::Publisher veh_path_pub_;
  ros::Publisher control_vcu_pub_;
};
}
#endif
