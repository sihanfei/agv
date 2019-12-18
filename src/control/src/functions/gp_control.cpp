#include "gp_control.h"
#include "control.h"

using namespace std;
using namespace control;

namespace control
{

GPControl::GPControl(ros::NodeHandle &nh) : nh_(nh)
{
  ROS_INFO("into");

  ROS_INFO("Start initialization...");
  // temp variables initialization
  start_tip_ = 1;
  math_tip_  = 1;
  plan_flag  = 1;
  read_path_flag = 1;
  num_ = 0;

  latteral_error           = 0.0;
  previous_control_angle_f = 0.0;
  previous_control_angle_r = 0.0;
  previous_endx = 0.0;
  previous_endy = 0.0;

  time_now      = 0.0;
  time_previous = 0.0;

  indexr1 = 0;
  indexr2 = 0;
  indexr3 = 0;
  indexf1 = 0;
  indexf2 = 0;
  indexf3 = 0;

  size_tmp = 0;
  ss = 0;

  // get paramaters from yaml
  ROS_INFO("Start to load paramaters...");
  nh.getParam("/control/mode", mode);
  nh.getParam("/control/pathtype", pathtype);
  nh.getParam("/control/lateral/kp", lateral_kp);
  nh.getParam("/control/lateral/ki", lateral_ki);
  nh.getParam("/control/lateral/kd", lateral_kd);
  nh.getParam("/control/straight/speed", straight_speed);
  nh.getParam("/control/speed/limit", speed_limit);
  nh.getParam("/control/Rmax", Rmax);
  nh.getParam("/control/turn_ratio", turn_ratio);
  nh.getParam("/control/max_turn_angle", max_turn_angle);
  nh.getParam("/control/center2frontaxis", center2frontaxis);
  nh.getParam("/control/center2rearaxis", center2rearaxis);
  nh.getParam("/control/insgps_x", insgps_x);
  nh.getParam("/control/insgps_y", insgps_y);
  nh.getParam("/control/turnleft_ispositive", turnleft_ispositive);
  nh.getParam("/control/preview_time", preview_time);
  nh.getParam("/control/basic_prelength_min", basic_prelength_min);
  nh.getParam("/control/basic_prelength_max", basic_prelength_max);
  nh.getParam("/control/lamda_preview", lamda_preview);
  nh.getParam("/control/lamda_stanley", lamda_stanley);
  nh.getParam("/control/k_stanley", k_stanley);
  nh.getParam("/control/k_soft", k_soft);
  nh.getParam("/control/yawrate_limit", yawrate_limit);
  nh.getParam("/control/sys_delay", sys_delay);
  nh.getParam("/control/start2stop_dist", start2stop_dist);
  nh.getParam("/control/equal_length", equal_length);
  nh.getParam("/control/L0", L0);
  nh.getParam("/control/lamda0", lamda0);
  nh.getParam("/control/hb", hb);
  ROS_INFO("Paramaters loading finished.");

  //发
  joint_pub_     = nh.advertise< sensor_msgs::JointState >("/control/joint_states", 1);
  g_gps_pub_     = nh.advertise< nav_msgs::Odometry >("/control/odom_msg", BUF_LEN, true);
  ref_point_pub_ = nh.advertise< geometry_msgs::PointStamped >("/control/ref_point_msg", BUF_LEN, true);
  pre_point_pub_ = nh.advertise< geometry_msgs::PointStamped >("/control/pre_point_msg", BUF_LEN, true);
  veh_point_pub_ = nh.advertise< geometry_msgs::PointStamped >("/control/veh_point_msg", BUF_LEN, true);
  veh_path_pub_  = nh.advertise< nav_msgs::Path >("/control/veh_path", BUF_LEN, true);

  if (mode == 0)
  {
    control_vcu_pub_ = nh.advertise< geometry_msgs::PoseStamped >("/control/control_agv", BUF_LEN, true);
  }
  else
  {
    control_vcu_pub_ = nh.advertise< control_msgs::ADControlAGV >("/control/control_agv", BUF_LEN, true);
  }

  //收
  // bestpos_sub_ = nh.subscribe("bestpos",10, &RTKControl::recvbestposCallback,this);
  // Inspva_sub_ = nh.subscribe("inspva",10, &RTKControl::recvInspvaCallback,this);
  // velocity_sub_ = nh.subscribe("com2vehmsg",10,&RTKControl::recvVehDataCallback,this);

  //pos_sub_ = nh.subscribe("/localization/fusion_msg", 10, &GPControl::recvFusionPosCallback, this);
  prescan_position_sub_ = nh.subscribe("/prescan/location_xyz",10,&GPControl::recvPrescanRealtimePosCallback,this);


  local_path_sub_ = nh.subscribe("/map/ref_point_info", 10, &GPControl::recvGlobalPathCallback, this);

  setAnglePID();
  setSpeedPID();
  ROS_INFO("Initialization finished.");

  GPControl::gpControl();
}

GPControl::~GPControl()
{
}

void GPControl::setAnglePID()
{
  pid_conf_angle.kp  = lateral_kp; // 0.35;
  pid_conf_angle.ki  = lateral_ki; // 0.01;
  pid_conf_angle.kd  = lateral_kd; // 0.05;
  pid_conf_angle.kaw = 0;
  pid_control_angle.init(pid_conf_angle);
}

void GPControl::setSpeedPID()
{
  pid_conf_speed.kp  = 1;
  pid_conf_speed.ki  = 0;
  pid_conf_speed.kd  = 0;
  pid_conf_speed.kaw = 0;
  pid_control_speed.init(pid_conf_speed);
}

void GPControl::gpControl()
{
  ROS_INFO("into-2");
  ros::Rate loop_rate(FRE);

  //临时变量
  positionConf real_pos_temp            = {0}; //组合导航传来的组合导航经纬度位姿
  positionConf pre_pos_temp             = {0}; //预瞄点的XYZ位姿
  positionConf xy_pos_temp              = {0}; //车辆中心全局XYZ位姿
  positionConf xy_pos_temp_front_center = {0}; //车辆前轴中心全局XYZ位姿
  positionConf vnxy_pos_temp            = {0}; //距车辆中心最近点的xyz车辆坐标系位姿
  positionConf vnxy_pos_temp_fc         = {0}; //距车辆前轴中心最近点的xyz车辆坐标系位姿
  positionConf vpxy_pos_temp            = {0}; //预瞄点的xyz车辆坐标系位姿

  while (ros::ok())
  {

    if (start_tip_ == 2 && math_tip_ == 2 && size_tmp > 0 && read_path_flag == 2)
    {

      real_pos_temp.lon                = real_position_.lon;
      real_pos_temp.lat                = real_position_.lat;
      real_pos_temp.height             = real_position_.height;
      real_pos_temp.heading            = real_position_.heading;
      real_pos_temp.pitch              = real_position_.pitch;
      real_pos_temp.roll               = real_position_.roll;
      real_pos_temp.velocity_x         = real_position_.velocity_x;
      real_pos_temp.velocity_y         = real_position_.velocity_y;
      real_pos_temp.velocity_z         = real_position_.velocity_z;
      real_pos_temp.gps_seconds        = real_position_.gps_seconds;
      real_pos_temp.n_gps_sequence_num = real_position_.n_gps_sequence_num;
      real_pos_temp.velocity           = real_position_.velocity;

      // ROS_INFO("debug1");

      //经纬度转换成局部坐标
      if (mode == 0)
      {
        prescanxy2xy(xy_pos_temp, real_pos_temp);
        // ROS_INFO("debug2");
      }
      else
      {
        gps2xy(xy_pos_temp, real_pos_temp);
      }

      //组合导航坐标转成车辆中心坐标
      insgps2center(xy_pos_temp);
      xy_pos_temp_front_center = xy_pos_temp;
      // ROS_INFO("debug3");

      //车辆中心坐标转成车辆前轴中心坐标
      center2frontaxis_tf(xy_pos_temp_front_center);
      // ROS_INFO("debug4");

      //最小索引 规划路径上的距离车辆中心的最接近点/距离车辆前轴中心的最接近点
      int min_index_theory              = findClosestRefPoint(xy_pos_temp);
      int min_index_theory_front_center = findClosestRefPoint(xy_pos_temp_front_center);
      // ROS_INFO("min_index_theory_front_center:%d",min_index_theory_front_center);
      // ROS_INFO("debug5");

      //考虑系统延迟推算真正的最近点以及计算曲率半径的点
      int min_index;
      int min_index_front_center;

      min_index              = findRealMinIndex(min_index_theory, xy_pos_temp);
      min_index_front_center = findRealMinIndex(min_index_theory_front_center, xy_pos_temp_front_center);
      findPoints2ComputeRadius(min_index, indexr1, indexf1, indexf2, indexf3);

      //两个最近点以及计算曲率半径的点
      positionConf &nearst_pos_temp    = route_data_[min_index];
      positionConf &nearst_pos_temp_fc = route_data_[min_index_front_center];
      positionConf &ir1                = route_data_[indexr1];
      positionConf &ir2                = route_data_[indexr2];
      positionConf &ir3                = route_data_[indexr3];
      positionConf &if1                = route_data_[indexf1];
      positionConf &if2                = route_data_[indexf2];
      positionConf &if3                = route_data_[indexf3];

      //计算曲率半径
      double R1    = getR(nearst_pos_temp, ir1, if1);
      double R4    = getR(nearst_pos_temp, if1, if2);
      double R5    = getR(if1, if2, if3);
      double R_tmp = getMinR(R1, R4, R5);
      // ROS_INFO("debug7");

      //计算前后轮转向比
      double sigma = ComputeSigma(R_tmp);
      // ROS_INFO("debug8");

      //计算预瞄点
      findPrePoint(pre_pos_temp, xy_pos_temp, min_index, R_tmp);
      // ROS_INFO("debug9");

      //将两个最近点和预瞄点的坐标转换到车辆坐标系下
      xy2vxy(vnxy_pos_temp, nearst_pos_temp, xy_pos_temp);
      // ROS_INFO("debug9-1");
      xy2vxy(vnxy_pos_temp_fc, nearst_pos_temp_fc, xy_pos_temp_front_center);
      // ROS_INFO("debug9-2");
      xy2vxy(vpxy_pos_temp, pre_pos_temp, xy_pos_temp);
      // ROS_INFO("debug10");

      //横向控制模型计算横向误差
      latteral_error = StanleyPreviewLatteralModel(xy_pos_temp, vnxy_pos_temp, vnxy_pos_temp_fc, vpxy_pos_temp);
      // ROS_INFO("debug11");

      double control_angle_f = 0;
      double control_angle_r = 0;
      control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
      control_angle_r        = -sigma * control_angle_f;

      //横摆角速度限制
      time_now = ros::Time::now().toSec();
      yawrate_constrain(time_now, time_previous, control_angle_f, previous_control_angle_f);
      yawrate_constrain(time_now, time_previous, control_angle_r, previous_control_angle_r);

      previous_control_angle_f = control_angle_f;
      previous_control_angle_r = control_angle_r;
      time_previous            = time_now;
      //最大转角限制
      angle_constrain(control_angle_f);
      angle_constrain(control_angle_r);
      // ROS_INFO("debug12");

      ROS_INFO("control_angle_f:%f min_index:%d min_index_front_center:%d R:%f sigma:%f e:%f", control_angle_f,
               min_index, min_index_front_center, R_tmp, sigma, vnxy_pos_temp.x);

      //发布控制命令，包括转向角，期望速度，期望加速度，车辆速度
      if (mode == 0)
      {
        geometry_msgs::PoseStamped prescan_control_command;
        if((ss - 1)<=min_index)
        {
           prescan_control_command.pose.orientation.x = 0;
           prescan_control_command.pose.orientation.z = 0;
        }
        else
        {
           prescan_control_command.pose.orientation.x = control_angle_f;
           prescan_control_command.pose.orientation.z = control_angle_r;
        }
        
        if((ss - min_index)*equal_length <= start2stop_dist)
        {
           prescan_control_command.pose.position.x = sqrt((route_data_[ss - 1].x - route_data_[min_index].x) *
                                                               (route_data_[ss - 1].x - route_data_[min_index].x) +
                                                           (route_data_[ss - 1].y - route_data_[min_index].y) *
                                                               (route_data_[ss - 1].y - route_data_[min_index].y)) *
                                                      (speed_limit / start2stop_dist);
        }
        else
        {
           prescan_control_command.pose.position.x = computespeed(R_tmp, straight_speed);
        }

        if(prescan_control_command.pose.position.x > speed_limit)
        {
          prescan_control_command.pose.position.x = speed_limit;
        }
        // prescan_control_command.pose.orientation.x =xy_pos_temp.x;
        // prescan_control_command.pose.orientation.y =xy_pos_temp.y;
        // prescan_control_command.pose.orientation.z =vnxy_pos_temp.x;
        // prescan_control_command.pose.orientation.w = min_index;
        control_vcu_pub_.publish(prescan_control_command);
      }
      else
      {
        control_msgs::ADControlAGV control_msg;
        control_msg.VehAgl_F = control_angle_f;
        control_msg.VehAgl_R = control_angle_r;

        if (plan_flag == 0)
        {
          control_msg.Vel_Req  = 0;
          control_msg.VehAgl_F = 0;
          control_msg.VehAgl_R = 0;
          // ROS_INFO("In the stop mode----expected speed:%f min_index:%d
          // min_index_front_center:%d",control_msg.ExpSpeed,min_index,min_index_front_center);
        }
        else
        {
          control_msg.Vel_Req = nearst_pos_temp.velocity;
          // ROS_INFO("control_angle_f:%f min_index:%d min_index_front_center:%d R:%f sigma:%f
          // e:%f",control_angle_f,min_index,min_index_front_center,R_tmp,sigma,vnxy_pos_temp.x);
        }

        // control_msg.ACCexp = 0;
        // control_msg.vehicle_x =xy_pos_temp.x;
        // control_msg.vehicle_y =xy_pos_temp.y;
        // control_msg.vehicle_err =vnxy_pos_temp.x;
        // control_msg.min_index = min_index;
        control_vcu_pub_.publish(control_msg);
      }

      //发布车辆实际运动路径
      
      this_pose_stamped.pose.position.x = xy_pos_temp.x;
      this_pose_stamped.pose.position.y = xy_pos_temp.y;
      this_pose_stamped.header.stamp = ros::Time::now();
      this_pose_stamped.header.frame_id = "odom";

      //path.header.stamp = ros::Time::now();
      path.header.frame_id = "odom";
      path.poses.push_back(this_pose_stamped);
      if(num_ >= 10)
      {
        veh_path_pub_.publish(path);
        num_ = 0;
      }

      //发布车辆前轴中心点，最近点和预瞄点 用于rviz显示
      geometry_msgs::PointStamped psmsg_veh;
      psmsg_veh.header.stamp    = ros::Time::now();
      psmsg_veh.header.frame_id = "odom";
      psmsg_veh.point.x         = xy_pos_temp.x;
      psmsg_veh.point.y         = xy_pos_temp.y;
      psmsg_veh.point.z         = 0;
      veh_point_pub_.publish(psmsg_veh);

      geometry_msgs::PointStamped psmsg;
      psmsg.header.stamp    = ros::Time::now();
      psmsg.header.frame_id = "odom";
      psmsg.point.x         = route_data_[min_index].x;
      psmsg.point.y         = route_data_[min_index].y;
      psmsg.point.z         = 0;
      ref_point_pub_.publish(psmsg);

      geometry_msgs::PointStamped psmsg_gps;
      psmsg_gps.header.stamp    = ros::Time::now();
      psmsg_gps.header.frame_id = "odom";
      psmsg_gps.point.x         = pre_pos_temp.x;
      psmsg_gps.point.y         = pre_pos_temp.y;
      psmsg_gps.point.z         = 0;
      pre_point_pub_.publish(psmsg_gps);

      start_tip_ = 1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

double GPControl::getR(const positionConf &pt1, const positionConf &pt2, const positionConf &pt3)
{
  double a  = 2 * (pt2.x - pt1.x);
  double b  = 2 * (pt2.y - pt1.y);
  double c  = pt2.x * pt2.x + pt2.y * pt2.y - pt1.x * pt1.x - pt1.y * pt1.y;
  double d  = 2 * (pt3.x - pt2.x);
  double ee = 2 * (pt3.y - pt2.y);
  double f  = pt3.x * pt3.x + pt3.y * pt3.y - pt2.x * pt2.x - pt2.y * pt2.y;
  double r;
  if ((b * d - ee * a) == 0)
  {
    r = 500;
  }
  else
  {
    double x = (b * f - ee * c) / (b * d - ee * a);
    double y = (d * c - a * f) / (b * d - ee * a);
    r        = sqrt((x - pt1.x) * (x - pt1.x) + (y - pt1.y) * (y - pt1.y));
    if (r >= 500)
    {
      r = 500;
    }
  }

  return r;
}

double GPControl::ComputeSigma(const double &R)
{
  double sig;
  double Rmin = (center2frontaxis + center2rearaxis) / (2 * sin(max_turn_angle * M_PI / 180));
  if (R <= Rmin)
  {
    sig = 1;
  }
  else if (R > Rmin && R < 500)
  {
    sig = -2 / M_PI * atan(0.01 * (R - Rmin)) + 1;
  }
  else
  {
    sig = 0;
  }
  return sig;
}

double GPControl::getMinR(const double &R1, const double &R4, const double &R5)
{
  double min_tmp;
  if (R1 <= R4)
  {
    min_tmp = R1;
  }
  else
  {
    min_tmp = R4;
  }

  if (min_tmp > R5)
  {
    min_tmp = R5;
  }

  return min_tmp;
}

void GPControl::findPoints2ComputeRadius(const int &index, int &index_r1, int &index_f1, int &index_f2, int &index_f3)
{
  double dl = (center2frontaxis + center2rearaxis) / 2;
  int s     = route_data_.size();

  // find rear point
  double l         = 0;
  int iter_index_r = index - 1;
  while (l < dl)
  {
    if (iter_index_r < 0)
    {
      l            = dl;
      iter_index_r = -1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_r].x - route_data_[iter_index_r + 1].x) *
                       (route_data_[iter_index_r].x - route_data_[iter_index_r + 1].x) +
                   (route_data_[iter_index_r].y - route_data_[iter_index_r + 1].y) *
                       (route_data_[iter_index_r].y - route_data_[iter_index_r + 1].y));
      iter_index_r = iter_index_r - 1;
    }
  }
  index_r1 = iter_index_r + 1;

  // find front three points
  l                = 0;
  int iter_index_f = index + 1;
  while (l < dl)
  {
    if (iter_index_f > (s - 1))
    {
      l            = dl;
      iter_index_f = s;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y));
      iter_index_f = iter_index_f + 1;
    }
  }
  index_f1 = iter_index_f - 1;

  l            = 0;
  iter_index_f = index + 1;
  while (l < 2 * dl)
  {
    if (iter_index_f > (s - 1))
    {
      l            = 2 * dl;
      iter_index_f = s;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y));
      iter_index_f = iter_index_f + 1;
    }
  }
  index_f2 = iter_index_f - 1;

  l            = 0;
  iter_index_f = index + 1;
  while (l < 3 * dl)
  {
    if (iter_index_f > (s - 1))
    {
      l            = 3 * dl;
      iter_index_f = s;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y));
      iter_index_f = iter_index_f + 1;
    }
  }
  index_f3 = iter_index_f - 1;
}

void GPControl::findPoints2ComputeRadius_ringpath(const int &index, int &index_r1, int &index_f1, int &index_f2,
                                                  int &index_f3)
{
  double dl = (center2frontaxis + center2rearaxis) / 2;
  int s     = route_data_.size();

  // find rear point
  double l         = 0;
  int iter_index_r = index - 1;
  while (l < dl)
  {
    if (iter_index_r < 0)
    {
      iter_index_r = s - 1;
      l = l + sqrt((route_data_[iter_index_r].x - route_data_[0].x) * (route_data_[iter_index_r].x - route_data_[0].x) +
                   (route_data_[iter_index_r].y - route_data_[0].y) * (route_data_[iter_index_r].y - route_data_[0].y));
      ;
      iter_index_r = iter_index_r - 1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_r].x - route_data_[iter_index_r + 1].x) *
                       (route_data_[iter_index_r].x - route_data_[iter_index_r + 1].x) +
                   (route_data_[iter_index_r].y - route_data_[iter_index_r + 1].y) *
                       (route_data_[iter_index_r].y - route_data_[iter_index_r + 1].y));
      iter_index_r = iter_index_r - 1;
    }
  }
  index_r1 = iter_index_r + 1;

  // find front three points
  l                = 0;
  int iter_index_f = index + 1;
  while (l < dl)
  {
    if (iter_index_f > (s - 1))
    {
      iter_index_f = 0;
      l            = l + sqrt((route_data_[iter_index_f].x - route_data_[s - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[s - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[s - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[s - 1].y));
      iter_index_f = iter_index_f + 1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y));
      iter_index_f = iter_index_f + 1;
    }
  }
  index_f1 = iter_index_f - 1;

  l            = 0;
  iter_index_f = index + 1;
  while (l < 2 * dl)
  {
    if (iter_index_f > (s - 1))
    {
      iter_index_f = 0;
      l            = l + sqrt((route_data_[iter_index_f].x - route_data_[s - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[s - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[s - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[s - 1].y));
      iter_index_f = iter_index_f + 1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y));
      iter_index_f = iter_index_f + 1;
    }
  }
  index_f2 = iter_index_f - 1;

  l            = 0;
  iter_index_f = index + 1;
  while (l < 3 * dl)
  {
    if (iter_index_f > (s - 1))
    {
      iter_index_f = 0;
      l            = l + sqrt((route_data_[iter_index_f].x - route_data_[s - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[s - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[s - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[s - 1].y));
      iter_index_f = iter_index_f + 1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) *
                       (route_data_[iter_index_f].x - route_data_[iter_index_f - 1].x) +
                   (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y) *
                       (route_data_[iter_index_f].y - route_data_[iter_index_f - 1].y));
      iter_index_f = iter_index_f + 1;
    }
  }
  index_f3 = iter_index_f - 1;
}

void GPControl::angle_constrain(double &input_value)
{
  if (input_value <= -max_turn_angle)
  {
    input_value = -max_turn_angle;
  }
  else if (input_value >= max_turn_angle)
  {
    input_value = max_turn_angle;
  }
  else
  {
    input_value = input_value;
  }
}

void GPControl::yawrate_constrain(const double &timenow, const double &timeprevious, double &control_angle,
                                  double &control_angle_p)
{
  if (time_previous == 0)
  {
    control_angle = control_angle;
  }
  else
  {
    if ((control_angle - control_angle_p) >= yawrate_limit * (timenow - timeprevious))
    {
      control_angle = control_angle_p + yawrate_limit * (timenow - timeprevious);
    }
    else if ((control_angle - control_angle_p) <= -yawrate_limit * (timenow - timeprevious))
    {
      control_angle = control_angle_p - yawrate_limit * (timenow - timeprevious);
    }
    else
    {
      control_angle = control_angle;
    }
  }
}

void GPControl::gps2xy(positionConf &xy_p, const positionConf &real_p)
{
  double RE0 = R0 / (sqrt(1 - e * e * sin(L0 * M_PI / 180) * sin(L0 * M_PI / 180)));
  double x0  = (RE0 + hb) * cos(L0 * M_PI / 180) * cos(lamda0 * M_PI / 180);
  double y0  = (RE0 + hb) * cos(L0 * M_PI / 180) * sin(lamda0 * M_PI / 180);
  double z0  = ((1 - e * e) * RE0 + hb) * sin(L0 * M_PI / 180);

  double L     = real_p.lat;
  double lamda = real_p.lon;
  double h     = real_p.height;
  double RE    = R0 / (sqrt(1 - e * e * sin(L * M_PI / 180) * sin(L * M_PI / 180)));
  double dx    = (RE + h) * cos(L * M_PI / 180) * cos(lamda * M_PI / 180) - x0;
  double dy    = (RE + h) * cos(L * M_PI / 180) * sin(lamda * M_PI / 180) - y0;
  double dz    = ((1 - e * e) * RE + h) * sin(L * M_PI / 180) - z0;

  double dn = -sin(L * M_PI / 180) * cos(lamda * M_PI / 180)*dx - sin(L * M_PI / 180) * sin(lamda * M_PI / 180) * dy +
              cos(L * M_PI / 180) * dz;
  double de = -sin(lamda * M_PI / 180) * dx + cos(lamda * M_PI / 180) * dy;
  double dd = -cos(L * M_PI / 180) * cos(lamda * M_PI / 180) * dx - cos(L * M_PI / 180) * sin(lamda * M_PI / 180) * dy -
              sin(L * M_PI / 180) * dz;

  xy_p.x          = de;
  xy_p.y          = dn;
  xy_p.z          = -dd;
  xy_p.lon        = real_p.lon;
  xy_p.lat        = real_p.lat;
  xy_p.height     = real_p.height;
  xy_p.velocity   = real_p.velocity;
  xy_p.velocity_x = real_p.velocity_x;
  xy_p.velocity_y = real_p.velocity_y;
  xy_p.velocity_z = real_p.velocity_z;
  xy_p.heading    = real_p.heading;
  xy_p.pitch      = real_p.pitch;
  xy_p.roll       = real_p.roll;
}

void GPControl::prescanxy2xy(positionConf &xy_p, const positionConf &real_p)
{
  xy_p.x          = real_p.lon;
  xy_p.y          = real_p.lat;
  xy_p.z          = real_p.height;
  xy_p.lon        = real_p.lon;
  xy_p.lat        = real_p.lat;
  xy_p.height     = real_p.height;
  xy_p.velocity   = real_p.velocity;
  xy_p.velocity_x = real_p.velocity_x;
  xy_p.velocity_y = real_p.velocity_y;
  xy_p.velocity_z = real_p.velocity_z;
  xy_p.heading    = real_p.heading;
  xy_p.pitch      = real_p.pitch;
  xy_p.roll       = real_p.roll;
}

void GPControl::xy2vxy(positionConf &targetp_veh, const positionConf &targetp_xy, const positionConf &xy_p)
{
  targetp_veh.x = (targetp_xy.x - xy_p.x) * cos(xy_p.heading * M_PI / 180) -
                  (targetp_xy.y - xy_p.y) * sin(xy_p.heading * M_PI / 180);
  targetp_veh.y = (targetp_xy.x - xy_p.x) * sin(xy_p.heading * M_PI / 180) +
                  (targetp_xy.y - xy_p.y) * cos(xy_p.heading * M_PI / 180);
  targetp_veh.z = targetp_xy.z;

  targetp_veh.velocity   = targetp_xy.velocity;
  targetp_veh.velocity_x = targetp_xy.velocity_x;
  targetp_veh.velocity_y = targetp_xy.velocity_y;
  targetp_veh.velocity_z = targetp_xy.velocity_z;
  targetp_veh.heading    = targetp_xy.heading;
  targetp_veh.pitch      = targetp_xy.pitch;
  targetp_veh.roll       = targetp_xy.roll;
}

double GPControl::computespeed(const double &R, const double &straightline_speed)
{
  return 2 / M_PI * atan(R / 50) * straightline_speed;
}

double GPControl::StanleyPreviewLatteralModel(const positionConf &xy_p, const positionConf &vnxy_p,
                                              const positionConf &vnxy_p_fc, const positionConf &vpxy_p)
{
  double D_err = 0;

  // calculate yaw
  double yaw = 0;
  if (xy_p.heading > (vnxy_p.heading + 180))
  {
    yaw = xy_p.heading - vnxy_p.heading - 360;
  }
  else if (xy_p.heading < (vnxy_p.heading - 180))
  {
    yaw = 360 - (vnxy_p.heading - xy_p.heading);
  }
  else
  {
    yaw = xy_p.heading - vnxy_p.heading;
  }

  // calculate preyaw
  double preyaw = 0;
  if (vpxy_p.x == 0)
  {
    if (vpxy_p.y >= 0)
    {
      preyaw = 0;
    }
    else
    {
      preyaw = 180;
    }
  }
  else
  {
    if (vpxy_p.x > 0 && vpxy_p.y >= 0)
    {
      preyaw = -atan(vpxy_p.x / vpxy_p.y) * 180 / M_PI;
    }
    else if (vpxy_p.x > 0 && vpxy_p.y < 0)
    {
      preyaw = (atan(vpxy_p.x / vpxy_p.y) - M_PI / 2) * 180 / M_PI;
    }
    else if (vpxy_p.x < 0 && vpxy_p.y >= 0)
    {
      preyaw = -atan(vpxy_p.x / vpxy_p.y) * 180 / M_PI;
    }
    else
    {
      preyaw = (atan(vpxy_p.x / vpxy_p.y) + M_PI / 2) * 180 / M_PI;
    }
  }

  double lamda_stanley_;

  if (xy_p.velocity <= 2.222)
  {
    lamda_stanley_ = lamda_stanley;
  }
  else if (xy_p.velocity > 2.222 && xy_p.velocity <= 5.555)
  {
    lamda_stanley_ = (-lamda_stanley / 3.333) * xy_p.velocity + (5 * lamda_stanley / 3);
  }
  else
  {
    lamda_stanley_ = 0;
  }
  lamda_stanley_ = lamda_stanley;
  if (turnleft_ispositive == 1)
  {
    D_err = yaw + lamda_preview * preyaw +
            lamda_stanley_ * atan2(-k_stanley * vnxy_p_fc.x, (k_soft + xy_p.velocity)) * 180 / M_PI;
  }
  else
  {
    D_err = -(yaw + lamda_preview * preyaw +
              lamda_stanley_ * atan2(-k_stanley * vnxy_p_fc.x, (k_soft + xy_p.velocity)) * 180 / M_PI);
  }
  //ROS_INFO("D_err:%f yaw:%f preyaw:%f stanley:%f vnxy_p.heading:%f xy_p.heading:%f",D_err,yaw,preyaw,atan2(-k_stanley*vnxy_p_fc.x,(k_soft + xy_p.velocity))*180/M_PI,vnxy_p.heading,xy_p.heading);
  return D_err;
}

void GPControl::findPrePoint(positionConf &pre_p, const positionConf &real_p, const int &index, const double &R)
{
  double pre_length;
  /*
  if(real_p.velocity <= 2.222)
  {
    pre_length = basic_preview_length;
  }
  else
  {
    pre_length = (real_p.velocity - 2.222)*preview_time + basic_preview_length;
  }
  */
  // double pre_length = basic_preview_length;

  pre_length = (2 * (basic_prelength_max - basic_prelength_min)) / (1 + exp(-(R - Rmax))) + basic_prelength_min +
               real_p.velocity * preview_time;

  double l       = 0;
  int s          = route_data_.size();
  int iter_index = index + 1;
  while (l < pre_length)
  {
    if (iter_index >= (s - 1))
    {
      iter_index = s;
      l          = pre_length;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index].x - route_data_[iter_index - 1].x) *
                       (route_data_[iter_index].x - route_data_[iter_index - 1].x) +
                   (route_data_[iter_index].y - route_data_[iter_index - 1].y) *
                       (route_data_[iter_index].y - route_data_[iter_index - 1].y));
      iter_index = iter_index + 1;
    }
  }
  pre_p.x          = route_data_[iter_index - 1].x;
  pre_p.y          = route_data_[iter_index - 1].y;
  pre_p.z          = route_data_[iter_index - 1].z;
  pre_p.heading    = route_data_[iter_index - 1].heading;
  pre_p.pitch      = route_data_[iter_index - 1].pitch;
  pre_p.roll       = route_data_[iter_index - 1].roll;
  pre_p.velocity   = route_data_[iter_index - 1].velocity;
  pre_p.velocity_x = route_data_[iter_index - 1].velocity_x;
  pre_p.velocity_y = route_data_[iter_index - 1].velocity_y;
  pre_p.velocity_z = route_data_[iter_index - 1].velocity_z;
}

void GPControl::findPrePoint_ringpath(positionConf &pre_p, const positionConf &real_p, const int &index,
                                      const double &R)
{
  double pre_length;
  /*
  if(real_p.velocity <= 2.222)
  {
    pre_length = basic_preview_length;
  }
  else
  {
    pre_length = (real_p.velocity - 2.222)*preview_time + basic_preview_length;
  }
  */
  // double pre_length = basic_preview_length;

  pre_length = (2 * (basic_prelength_max - basic_prelength_min)) / (1 + exp(-(R - Rmax))) + basic_prelength_min +
               real_p.velocity * preview_time;

  double l       = 0;
  int s          = route_data_.size();
  int iter_index = index + 1;
  while (l < pre_length)
  {
    if (iter_index > (s - 1))
    {
      iter_index = 0;
      l          = l +
          sqrt((route_data_[iter_index].x - route_data_[s - 1].x) * (route_data_[iter_index].x - route_data_[s - 1].x) +
               (route_data_[iter_index].y - route_data_[s - 1].y) * (route_data_[iter_index].y - route_data_[s - 1].y));
      iter_index = iter_index + 1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index].x - route_data_[iter_index - 1].x) *
                       (route_data_[iter_index].x - route_data_[iter_index - 1].x) +
                   (route_data_[iter_index].y - route_data_[iter_index - 1].y) *
                       (route_data_[iter_index].y - route_data_[iter_index - 1].y));
      iter_index = iter_index + 1;
    }
  }
  pre_p.x          = route_data_[iter_index - 1].x;
  pre_p.y          = route_data_[iter_index - 1].y;
  pre_p.z          = route_data_[iter_index - 1].z;
  pre_p.heading    = route_data_[iter_index - 1].heading;
  pre_p.pitch      = route_data_[iter_index - 1].pitch;
  pre_p.roll       = route_data_[iter_index - 1].roll;
  pre_p.velocity   = route_data_[iter_index - 1].velocity;
  pre_p.velocity_x = route_data_[iter_index - 1].velocity_x;
  pre_p.velocity_y = route_data_[iter_index - 1].velocity_y;
  pre_p.velocity_z = route_data_[iter_index - 1].velocity_z;
}

int GPControl::findRealMinIndex(const int &min_index_theory_, const positionConf &real_p)
{
  double delay_length = real_p.velocity * sys_delay;
  double l            = 0;
  int s               = route_data_.size();
  int iter_index      = min_index_theory_ + 1;

  while (l < delay_length)
  {
    if (iter_index >= (s - 1))
    {
      iter_index = s;
      l          = delay_length;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index].x - route_data_[iter_index - 1].x) *
                       (route_data_[iter_index].x - route_data_[iter_index - 1].x) +
                   (route_data_[iter_index].y - route_data_[iter_index - 1].y) *
                       (route_data_[iter_index].y - route_data_[iter_index - 1].y));
      iter_index = iter_index + 1;
    }
  }
  return (iter_index - 1);
}

int GPControl::findRealMinIndex_ringpath(const int &min_index_theory_, const positionConf &real_p)
{
  double delay_length = real_p.velocity * sys_delay;
  double l            = 0;
  int s               = route_data_.size();
  int iter_index      = min_index_theory_ + 1;

  while (l < delay_length)
  {
    if (iter_index > (s - 1))
    {
      iter_index = 0;
      l          = l +
          sqrt((route_data_[iter_index].x - route_data_[s - 1].x) * (route_data_[iter_index].x - route_data_[s - 1].x) +
               (route_data_[iter_index].y - route_data_[s - 1].y) * (route_data_[iter_index].y - route_data_[s - 1].y));
      iter_index = iter_index + 1;
    }
    else
    {
      l = l + sqrt((route_data_[iter_index].x - route_data_[iter_index - 1].x) *
                       (route_data_[iter_index].x - route_data_[iter_index - 1].x) +
                   (route_data_[iter_index].y - route_data_[iter_index - 1].y) *
                       (route_data_[iter_index].y - route_data_[iter_index - 1].y));
      iter_index = iter_index + 1;
    }
  }
  return (iter_index - 1);
}

void GPControl::insgps2center(positionConf &xy_p)
{
  xy_p.x = xy_p.x - (insgps_x * cos(xy_p.heading * M_PI / 180) + insgps_y * sin(xy_p.heading * M_PI / 180));
  xy_p.y = xy_p.y - (insgps_y * cos(xy_p.heading * M_PI / 180) - insgps_x * sin(xy_p.heading * M_PI / 180));
}

void GPControl::center2frontaxis_tf(positionConf &xy_p)
{
  xy_p.x = xy_p.x + center2frontaxis * sin(xy_p.heading * M_PI / 180);
  xy_p.y = xy_p.y + center2frontaxis * cos(xy_p.heading * M_PI / 180);
}

double GPControl::pointDistanceSquare(const positionConf &xy_p, const positionConf &route_point)
{
  const double dx = xy_p.x - route_point.x;
  const double dy = xy_p.y - route_point.y;
  return dx * dx + dy * dy;
}

double GPControl::norm2(const double &dx, const double &dy)
{
  return sqrt(dx * dx + dy * dy);
}

int GPControl::findClosestRefPoint(const positionConf &rpt)
{
  float min_value = 1000000; // 1km范围内
  double ans      = 100000;
  int loc_num     = route_data_.size() + 1;
  // ROS_INFO("goto Finding %f %f\n",rpt.lon,rpt.lat);
  for (int i = 0; i < route_data_.size(); i++)
  {
    ans = pointDistanceSquare(rpt, route_data_[i]);

    if (ans == 0)
    {
      min_value = ans;
      loc_num   = i;
      // break;
    }

    if (ans < min_value)
    {
      min_value = ans;
      loc_num   = i;
    }
  }

  if (loc_num > route_data_.size())
  {
    ROS_INFO("Car is out of this map!!!\n");
    ROS_INFO("shutting down!\n");
    ros::shutdown();
  }

  return loc_num;
}

void GPControl::recvHuacePosCallback(const location_sensor_msgs::IMUAndGNSSInfo &msg)
{
  real_position_.velocity_x = msg.velocity.x;
  real_position_.velocity_y = msg.velocity.y;
  real_position_.velocity_z = msg.velocity.z;
  real_position_.velocity =
      sqrt(msg.velocity.x * msg.velocity.x + msg.velocity.y * msg.velocity.y + msg.velocity.z * msg.velocity.z);

  real_position_.heading = msg.yaw;
  real_position_.pitch   = msg.pitch;
  real_position_.roll    = msg.roll;

  real_position_.lon    = msg.pose.y;
  real_position_.lat    = msg.pose.x;
  real_position_.height = msg.pose.z;

  real_position_.gps_seconds        = msg.GPS_sec;
  real_position_.n_gps_sequence_num = msg.GPS_week;

  start_tip_ = 2;
}

double GPControl::sumSquares3D(const double &in_x, const double &in_y, const double &in_z)
{
  return (in_x * in_x + in_y * in_y + in_z * in_z);
}

void GPControl::recvFusionPosCallback(const location_msgs::FusionDataInfo &msg)
{
  real_position_.velocity_x = msg.velocity.linear.x;
  real_position_.velocity_y = msg.velocity.linear.y;
  real_position_.velocity_z = msg.velocity.linear.z;
  real_position_.velocity =
      sqrt(sumSquares3D(real_position_.velocity_x, real_position_.velocity_y, real_position_.velocity_z));

  real_position_.heading = msg.yaw;
  real_position_.pitch   = msg.pitch;
  real_position_.roll    = msg.roll;

  real_position_.lon    = msg.pose.x;
  real_position_.lat    = msg.pose.y;
  real_position_.height = msg.pose.z;

  real_position_.gps_seconds        = msg.sec;
  real_position_.n_gps_sequence_num = msg.day;

  start_tip_ = 2;
}

void GPControl::recvPrescanRealtimePosCallback(const nav_msgs::Odometry &msg)
{
  real_position_.velocity_x = 0;
  real_position_.velocity_y = 0;
  real_position_.velocity_z = 0;
  real_position_.velocity   = msg.twist.twist.linear.x;

  real_position_.heading = msg.twist.twist.angular.x;
  real_position_.pitch   = 0;
  real_position_.roll    = 0;

  real_position_.lon    = msg.pose.pose.position.x;
  real_position_.lat    = msg.pose.pose.position.y;
  real_position_.height = msg.pose.pose.position.z;

  real_position_.gps_seconds        = 0;
  real_position_.n_gps_sequence_num = 0;

  start_tip_ = 2;
  num_++;
}

void GPControl::recvGlobalPathCallback(const map_msgs::REFPointArray &msg)
{
  //判断全局路径是否更新
  size_tmp = msg.REF_line_INFO.size();
  if((msg.REF_line_INFO[size_tmp-1].rx != previous_endx) || (msg.REF_line_INFO[size_tmp-1].ry != previous_endy))
  {
     read_path_flag = 1;  
  }
  else
  {
     read_path_flag = 2;
  }
  //ROS_INFO("size of global path: %d, read path flag: %d",size_tmp,read_path_flag);  

  if(read_path_flag == 1)
  {
	positionConf read_position = {0};
	global_route_data_.clear();
	route_data_.clear();

	for (int i = 0; i < msg.REF_line_INFO.size(); i++)
	{
		read_position.x        = msg.REF_line_INFO[i].rx;
		read_position.y        = msg.REF_line_INFO[i].ry;
		read_position.heading  = msg.REF_line_INFO[i].rtheta;
		//read_position.velocity = msg.path_data_REF[i].v;

		global_route_data_.push_back(read_position);
	}
	//plan_flag = msg.path_plan_valid;

	//将传来的路径点插值变密
	int s = global_route_data_.size();
	for (int i = 0; i < (s - 1); i++)
	{
		route_data_.push_back(global_route_data_[i]);
                
		double l = sqrt((global_route_data_[i].x - global_route_data_[i + 1].x) *
				(global_route_data_[i].x - global_route_data_[i + 1].x) +
			    (global_route_data_[i].y - global_route_data_[i + 1].y) *
				(global_route_data_[i].y - global_route_data_[i + 1].y));
		int n            = ceil(l / equal_length);
		double dx        = (global_route_data_[i + 1].x - global_route_data_[i].x) / n;
		double dy        = (global_route_data_[i + 1].y - global_route_data_[i].y) / n;
		double dheading  = (global_route_data_[i + 1].heading - global_route_data_[i].heading) / n;
		//double dvelocity = (global_route_data_[i + 1].velocity - global_route_data_[i].velocity) / n;
		for (int j = 0; j < (n - 1); j++)
		{
			read_position.x        = global_route_data_[i].x + j * dx;
			read_position.y        = global_route_data_[i].y + j * dy;
			read_position.heading  = global_route_data_[i].heading + j * dheading;
			//read_position.velocity = global_route_data_[i].velocity + j * dvelocity;

			route_data_.push_back(read_position);
		}
                
	}
	route_data_.push_back(global_route_data_[s - 1]);

        ss = route_data_.size();
        previous_endx = msg.REF_line_INFO[size_tmp-1].rx;
        previous_endy = msg.REF_line_INFO[size_tmp-1].ry;
	math_tip_ = 2;
        read_path_flag = 2;
  }
}

} // namespace control
