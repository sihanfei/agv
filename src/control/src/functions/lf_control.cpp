#include "lf_control.h"
#include "control.h"

using namespace std;
using namespace control;

namespace control
{

LFControl::LFControl(ros::NodeHandle &nh) : nh_(nh)
{
  ROS_INFO("into");

  ROS_INFO("Start initialization...");
  // temp variables initialization
  start_tip_ = 1;
  math_tip_  = 1;
  plan_flag  = 1;
  path_mode  = 0;

  latteral_error           = 0.0;
  previous_control_angle_f = 0.0;
  previous_control_angle_r = 0.0;

  time_now      = 0.0;
  time_previous = 0.0;

  indexr1 = 0;
  indexr2 = 0;
  indexr3 = 0;
  indexf1 = 0;
  indexf2 = 0;
  indexf3 = 0;

  section_decide_num = 0;
  previous_section_decide_num = 0;

  // get paramaters from yaml
  ROS_INFO("Start to load paramaters...");
  nh.getParam("/control/mode", mode);
  nh.getParam("/control/pathtype", pathtype);
  nh.getParam("/control/lateral/kp", lateral_kp);
  nh.getParam("/control/lateral/ki", lateral_ki);
  nh.getParam("/control/lateral/kd", lateral_kd);
  nh.getParam("/control/straight/speed", straight_speed);
  nh.getParam("/control/speed/limit", speed_limit);
  nh.getParam("/control/tp_speed",tp_speed);
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
  nh.getParam("/control/refline_stop", refline_stop);
  ROS_INFO("Paramaters loading finished.");

  //发
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

  if(mode == 0)
  {
    pos_sub_ = nh.subscribe("/prescan/location_xyz", 10, &LFControl::recvPrescanRealtimePosCallback, this);
  }
  else
  {
    pos_sub_ = nh.subscribe("/localization/fusion_msg", 10, &LFControl::recvFusionPosCallback, this);
  }

  if(refline_stop == 1)
  {
    refline_stop_msg_sub_ = nh.subscribe("/map/stop_msg",10, &LFControl::recvReflineStopCallback,this);
    local_path_sub_ = nh.subscribe("/map/decision_info", 10, &LFControl::recvLocalPathCallback, this);
  }
  else
  {
    local_path_sub_ = nh.subscribe("/plan/decision_info", 10, &LFControl::recvLocalPathCallback, this);
  }

  setAnglePID();
  setSpeedPID();
  ROS_INFO("Initialization finished.");

  LFControl::lfControl();
}

LFControl::~LFControl()
{
}

void LFControl::setAnglePID()
{
  pid_conf_angle.kp  = lateral_kp; // 0.35;
  pid_conf_angle.ki  = lateral_ki; // 0.01;
  pid_conf_angle.kd  = lateral_kd; // 0.05;
  pid_conf_angle.kaw = 0;
  pid_control_angle.init(pid_conf_angle);
}

void LFControl::setSpeedPID()
{
  pid_conf_speed.kp  = 1;
  pid_conf_speed.ki  = 0;
  pid_conf_speed.kd  = 0;
  pid_conf_speed.kaw = 0;
  pid_control_speed.init(pid_conf_speed);
}

void LFControl::lfControl()
{
  ROS_INFO("into-2");
  ros::Rate loop_rate(FRE);

  //临时变量
  positionConf real_pos_temp            = {0}; //组合导航传来的组合导航经纬度位姿
  positionConf pre_pos_temp             = {0}; //预瞄点的XYZ位姿
  positionConf diag_pre_pos_temp        = {0}; //判断斜行点位姿
  positionConf xy_pos_temp              = {0}; //车辆中心全局XYZ位姿
  positionConf xy_pos_temp_front_center = {0}; //车辆前轴中心全局XYZ位姿
  positionConf vnxy_pos_temp            = {0}; //距车辆中心最近点的xyz车辆坐标系位姿
  positionConf vnxy_pos_temp_fc         = {0}; //距车辆前轴中心最近点的xyz车辆坐标系位姿
  positionConf vpxy_pos_temp            = {0}; //预瞄点的xyz车辆坐标系位姿

  while (ros::ok())
  {

    if (start_tip_ == 2 && math_tip_ == 2 && path_mode == 1)
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

      //经纬度转换成局部坐标
      if (mode == 0)
      {
        prescanxy2xy(xy_pos_temp, real_pos_temp);
      }
      else
      {
        prescanxy2xy(xy_pos_temp, real_pos_temp);
        //gps2xy(xy_pos_temp, real_pos_temp);
      }

      //组合导航坐标转成车辆中心坐标
      //insgps2center(xy_pos_temp);
      xy_pos_temp_front_center = xy_pos_temp;

      //查找奇点
      findStrangePoints();

      //路段截取
      route_cut(xy_pos_temp);

      //最小索引 规划路径上的距离车辆中心的最接近点/距离车辆前轴中心的最接近点
      int min_index_theory              = section_decide_num;

      //判断前进还是后退
      forward_backward_judge();

      ////车辆中心坐标转成车辆前轴中心坐标
      if(forward_backward_flag == -1)
      {
        xy_pos_temp.heading = xy_pos_temp.heading + 180;
        if(xy_pos_temp.heading > 360)
        {
          xy_pos_temp.heading = xy_pos_temp.heading - 360;
        }
      } 
      xy_pos_temp_front_center = xy_pos_temp;
      center2frontaxis_tf(xy_pos_temp_front_center);

      int min_index_theory_front_center = findClosestFCRefPoint(xy_pos_temp_front_center,min_index_theory);

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

      //计算预瞄点
      findPrePoint(pre_pos_temp, xy_pos_temp, min_index, R_tmp);
      findDiagPrePoint(diag_pre_pos_temp,xy_pos_temp,min_index,R_tmp);

      //将两个最近点和预瞄点的坐标转换到车辆坐标系下
      xy2vxy(vnxy_pos_temp, nearst_pos_temp, xy_pos_temp);
      xy2vxy(vnxy_pos_temp_fc, nearst_pos_temp_fc, xy_pos_temp_front_center);
      xy2vxy(vpxy_pos_temp, pre_pos_temp, xy_pos_temp);

      //横向控制模型计算横向误差
      latteral_error = StanleyPreviewLatteralModel(xy_pos_temp, vnxy_pos_temp, vnxy_pos_temp_fc, vpxy_pos_temp);

      //计算前后轮转向比
      if(forward_backward_flag == 1)
      {
        double sigma = ComputeSigma(R_tmp,nearst_pos_temp,xy_pos_temp,diag_pre_pos_temp,control_angle_f,control_angle_r);
      }
      else
      {
        double sigma = ComputeSigma(R_tmp,nearst_pos_temp,xy_pos_temp,diag_pre_pos_temp,control_angle_r,control_angle_f);
      }

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

      //发布控制命令，包括转向角，期望速度，期望加速度，车辆速度
      if (mode == 0)
      {
        geometry_msgs::PoseStamped prescan_control_command;
        prescan_control_command.pose.orientation.x = control_angle_f;
        prescan_control_command.pose.orientation.z = control_angle_r;
        prescan_control_command.pose.position.x    = nearst_pos_temp.velocity;
        
        if((-min_index_theory + route_section_index_[1]+1) <= 2)
        {
          ROS_INFO("In stop distance!");
          prescan_control_command.pose.orientation.x = 0;
          prescan_control_command.pose.orientation.z = 0;
        }
        if((-min_index_theory + route_section_index_[1]+1) <= 1)
        {
          prescan_control_command.pose.position.x    = 0;
        }
		/*
	if(plan_flag == 0)
	{
	prescan_control_command.pose.orientation.z = 0;
	prescan_control_command.pose.orientation.x = 0;
	prescan_control_command.pose.position.x = 0;
	//ROS_INFO("In the stop mode----expected speed:%f min_index:%d
	min_index_front_center:%d",prescan_control_command.pose.position.z,min_index,min_index_front_center);
	}
	else
	{
	prescan_control_command.pose.position.x = nearst_pos_temp.velocity;
	//ROS_INFO("control_angle_f:%f min_index:%d min_index_front_center:%d R:%f sigma:%f
	e:%f",control_angle_f,min_index,min_index_front_center,R_tmp,sigma,vnxy_pos_temp.x);
	}

	//speed limitation
	if(prescan_control_command.pose.position.x > speed_limit)
	{
	prescan_control_command.pose.position.x = speed_limit;
	}
	*/

        // velocity orientation
        prescan_control_command.pose.position.x = forward_backward_flag*prescan_control_command.pose.position.x;

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

        if(min_index >= (route_section_index_[1]+1 - 2))
        {
          control_msg.VehAgl_F = 0;
          control_msg.VehAgl_R = 0;
        }
        if(min_index >= (route_section_index_[1]+1 - 1))
        {
          control_msg.Vel_Req  = 0;
        }

        if (plan_flag == 0)
        {
          control_msg.Vel_Req  = 0;
          control_msg.VehAgl_F = 0;
          control_msg.VehAgl_R = 0;
        }
        else
        {
          control_msg.Vel_Req = nearst_pos_temp.velocity;
        }

        
        if(refline_stop == 1)
        {
          if(R_tmp >= 300)
          {
		  if(nearst_pos_temp.velocity >= refline_stop_msg.Vel_Req)
		  {
		    control_msg.Vel_Req = refline_stop_msg.Vel_Req;
		  }
		  control_msg.EStop = refline_stop_msg.EStop;
          }
        }
        
        // velocity orientation
        control_msg.Vel_Req = forward_backward_flag*control_msg.Vel_Req;

        // control_msg.ACCexp = 0;
        // control_msg.vehicle_x =xy_pos_temp.x;
        // control_msg.vehicle_y =xy_pos_temp.y;
        control_msg.vehicle_err =vnxy_pos_temp.x;
        control_msg.min_index = min_index;
        control_vcu_pub_.publish(control_msg);
      }

      //发布车辆实际运动路径
      /*
	this_pose_stamped.pose.position.x = xy_pos_temp.x;
	this_pose_stamped.pose.position.y = xy_pos_temp.y;
	this_pose_stamped.header.stamp = ros::Time::now();
	this_pose_stamped.header.frame_id = "odom";

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "odom";
	path.poses.push_back(this_pose_stamped);
	veh_path_pub_.publish(path);
      */

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

      previous_route_data_.clear();
      vector< positionConf >().swap(previous_route_data_);
      for(int i=0;i < route_data_.size();i++)
      {
	previous_route_data_.push_back(route_data_[i]);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void LFControl::forward_backward_judge()
{
  double dheading = computedheading(real_position_.heading,route_data_[section_decide_num].heading);
  if(abs(dheading) > 90)
  {
    forward_backward_flag = -1;
  }
  else
  {
    forward_backward_flag = 1;
  }
}

double LFControl::getR(const positionConf &pt1, const positionConf &pt2, const positionConf &pt3)
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

double LFControl::ComputeSigma(const double &R,const positionConf &_nearst_pos_,const positionConf &_xy_pos_,const positionConf &_pre_pos_,double &_control_angle_f,double &_control_angle_r)
{
  double sig;
  double Rmin = (center2frontaxis + center2rearaxis) / (2 * sin(max_turn_angle * M_PI / 180));
  double dheading_nearst_pre = computedheading(_nearst_pos_.heading,_pre_pos_.heading);
  if(abs(dheading_nearst_pre) <= 1)
  {
          double heading_n_pre = computeheading(_nearst_pos_,_pre_pos_);
          double dheading_n_pre = computedheading(heading_n_pre,_pre_pos_.heading);
          if(abs(dheading_n_pre) <= 1)
          {
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

            _control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
            _control_angle_r        = -sig * _control_angle_f;
          }
          else
          {
            ROS_INFO("Diagonal Control.");
            double dheading_v_n = computedheading(_xy_pos_.heading,_nearst_pos_.heading);
            if(abs(dheading_v_n) >= 20)
            {
              sig = 0;
            }
            else
            {
              sig = (-1.0/20.0)*abs(dheading_v_n) + 1;
            }

            if(turnleft_ispositive == 0)//左转为负
            {
              if(latteral_error < 0)
              {
                if(dheading_v_n <= 0)
                {
                  _control_angle_r        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_f        = sig*_control_angle_r;
                }
                else
                {
                  _control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_r        = sig*_control_angle_f;
                }
              }
              else
              {
                if(dheading_v_n <= 0)
                {
                  _control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_r        = sig*_control_angle_f;
                }
                else
                {
                  _control_angle_r        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_f        = sig*_control_angle_r;
                }
              }
            }
            else
            {
              if(latteral_error > 0)
              {
                if(dheading_v_n <= 0)
                {
                  _control_angle_r        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_f        = sig*_control_angle_r;
                }
                else
                {
                  _control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_r        = sig*_control_angle_f;
                }
              }
              else
              {
                if(dheading_v_n <= 0)
                {
                  _control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_r        = sig*_control_angle_f;
                }
                else
                {
                  _control_angle_r        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
                  _control_angle_f        = sig*_control_angle_r;
                }
              }
            }
          }
  }
  else
  {
          //ROS_INFO("Normal Control");
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
          _control_angle_f        = turn_ratio * pid_control_angle.control(latteral_error, 0.01);
          _control_angle_r        = -sig * _control_angle_f;
  }

  return sig;
}

double LFControl::computeheading(const positionConf &_p1_,const positionConf &_p2_)
{
  double heading = 0;
  if(_p1_.x == _p2_.x)
  {
    if(_p2_.y >= _p1_.y)
    {
      heading = 0;
    }
    else
    {
      heading = 180;
    }
  }
  else
  {
    if(atan((_p1_.y - _p2_.y)/(_p1_.x - _p2_.x)) == 0)
    {
      if(_p1_.x > _p2_.x)
      {
        heading = 270;
      }
      else
      {
        heading = 90;
      }
    }
    else
    {
      if(_p2_.x > _p1_.x)//1,4像限
      {
        if(_p2_.y > _p1_.y)//1
        {
          heading = 90 - atan((_p1_.y - _p2_.y)/(_p1_.x - _p2_.x))*180/M_PI;
        }
        else//4
        {
          heading = 90 - atan((_p1_.y - _p2_.y)/(_p1_.x - _p2_.x))*180/M_PI;
        }
      }
      else//2,3像限
      {
        if(_p2_.y > _p1_.y)//2
        {
          heading = 270 - atan((_p1_.y - _p2_.y)/(_p1_.x - _p2_.x))*180/M_PI;
        }
        else//3
        {
          heading = 270 - atan((_p1_.y - _p2_.y)/(_p1_.x - _p2_.x))*180/M_PI;
        }
      }
    }
  }
}

double LFControl::computedheading(const double &_heading1_,const double &_heading2_)
{
  double dheading = 0;
  if (_heading1_ > (_heading2_ + 180))
  {
    dheading = _heading1_ - _heading2_ - 360;
  }
  else if (_heading1_ < (_heading2_ - 180))
  {
    dheading = 360 - (_heading2_ - _heading1_);
  }
  else
  {
    dheading = _heading1_ - _heading2_;
  }

  return dheading;
}

double LFControl::getMinR(const double &R1, const double &R4, const double &R5)
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

void LFControl::findPoints2ComputeRadius(const int &index, int &index_r1, int &index_f1, int &index_f2, int &index_f3)
{
  double dl = (center2frontaxis + center2rearaxis) / 2;
  int s     = route_section_index_[1]+1;

  // find rear point
  double l         = 0;
  int iter_index_r = index - 1;
  while (l < dl)
  {
    if (iter_index_r < route_section_index_[0])
    {
      l            = dl;
      iter_index_r = route_section_index_[0]-1;
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

void LFControl::findPoints2ComputeRadius_ringpath(const int &index, int &index_r1, int &index_f1, int &index_f2,
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

void LFControl::angle_constrain(double &input_value)
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

void LFControl::yawrate_constrain(const double &timenow, const double &timeprevious, double &control_angle,
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

void LFControl::gps2xy(positionConf &xy_p, const positionConf &real_p)
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

void LFControl::prescanxy2xy(positionConf &xy_p, const positionConf &real_p)
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

void LFControl::xy2vxy(positionConf &targetp_veh, const positionConf &targetp_xy, const positionConf &xy_p)
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

double LFControl::computespeed(const double &R, const double &straightline_speed)
{
  return 2 / M_PI * atan(R / 50) * straightline_speed;
}

double LFControl::StanleyPreviewLatteralModel(const positionConf &xy_p, const positionConf &vnxy_p,
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
  //ROS_INFO("D_err:%f yaw:%f preyaw:%f stanley:%f",D_err,yaw,preyaw,atan2(-k_stanley*vnxy_p_fc.x,(k_soft + xy_p.velocity))*180/M_PI);
  return D_err;
}

void LFControl::findDiagPrePoint(positionConf &pre_p, const positionConf &real_p, const int &index, const double &R)
{
  double pre_length;

  pre_length = 5;

  double l       = 0;
  int s          = route_section_index_[1]+1;
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

void LFControl::findPrePoint(positionConf &pre_p, const positionConf &real_p, const int &index, const double &R)
{
  double pre_length;
  pre_length = (2 * (basic_prelength_max - basic_prelength_min)) / (1 + exp(-(R - Rmax))) + basic_prelength_min +
               real_p.velocity * preview_time;

  double l       = 0;
  int s          = route_section_index_[1]+1;
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

void LFControl::findPrePoint_ringpath(positionConf &pre_p, const positionConf &real_p, const int &index,
                                      const double &R)
{
  double pre_length;

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

int LFControl::findRealMinIndex(const int &min_index_theory_, const positionConf &real_p)
{
  double delay_length = real_p.velocity * sys_delay;
  double l            = 0;
  int s               = route_section_index_[1]+1;
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

int LFControl::findRealMinIndex_ringpath(const int &min_index_theory_, const positionConf &real_p)
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

void LFControl::insgps2center(positionConf &xy_p)
{
  xy_p.x = xy_p.x - (insgps_x * cos(xy_p.heading * M_PI / 180) + insgps_y * sin(xy_p.heading * M_PI / 180));
  xy_p.y = xy_p.y - (insgps_y * cos(xy_p.heading * M_PI / 180) - insgps_x * sin(xy_p.heading * M_PI / 180));
}

void LFControl::center2frontaxis_tf(positionConf &xy_p)
{
  xy_p.x = xy_p.x + center2frontaxis * sin(xy_p.heading * M_PI / 180);
  xy_p.y = xy_p.y + center2frontaxis * cos(xy_p.heading * M_PI / 180);
}

double LFControl::pointDistanceSquare(const positionConf &xy_p, const positionConf &route_point)
{
  const double dx = xy_p.x - route_point.x;
  const double dy = xy_p.y - route_point.y;
  return dx * dx + dy * dy;
}

double LFControl::norm2(const double &dx, const double &dy)
{
  return sqrt(dx * dx + dy * dy);
}

void LFControl::findStrangePoints()
{
  if(previous_route_data_.size() == route_data_.size() && previous_route_data_[0].x == route_data_[0].x && previous_route_data_[0].y == route_data_[0].y && previous_route_data_[previous_route_data_.size()-1].x == route_data_[route_data_.size()-1].x && previous_route_data_[previous_route_data_.size()-1].y == route_data_[route_data_.size()-1].y)//与上一帧路径相同
  {
    
  }
  else//与上一帧路径不同
  {
    strange_points.clear();
    vector< int >().swap(strange_points);
    for(int i=0;i< (route_data_.size()-1);i++)
    {
      double dheading_tmp = computedheading(route_data_[i].heading,route_data_[i+1].heading);
      if(abs(dheading_tmp) >= 90)
      {
        strange_points.push_back(i);    
      }
    }
  }
}

void LFControl::route_cut(const positionConf &rpt)
{
  if(previous_route_data_.size() == route_data_.size() && previous_route_data_[0].x == route_data_[0].x && previous_route_data_[0].y == route_data_[0].y && previous_route_data_[previous_route_data_.size()-1].x == route_data_[route_data_.size()-1].x && previous_route_data_[previous_route_data_.size()-1].y == route_data_[route_data_.size()-1].y)//与上一帧路径相同
  {
    route_section_data_.clear();
    vector< positionConf >().swap(route_section_data_);
    route_section_index_.clear();
    vector< int >().swap(route_section_index_);

    int search_maxend_num;
    int search_minstart_num;
    if(strange_points.size() == 0)
    {
      search_minstart_num = 0;
      search_maxend_num = route_data_.size() - 1;
    }
    else
    {
      for(int i=0; i <= strange_points.size(); i++)
      {
        if(i == 0)
        {
          if(previous_section_decide_num < (strange_points[i] - 1))
          {
            search_minstart_num = 0;
            search_maxend_num = strange_points[i];
          }
        }
        else if(i == strange_points.size())
        {
          if(previous_section_decide_num >= (strange_points[i-1] - 1))
          {
            search_minstart_num = strange_points[i-1] + 1;
            search_maxend_num = route_data_.size() - 1;
          }
        }
        else
        {
          if(previous_section_decide_num >= (strange_points[i-1] - 1) && previous_section_decide_num < (strange_points[i] - 1))
          {
            search_minstart_num = strange_points[i-1] + 1;
            search_maxend_num = strange_points[i];
          }
        }
      }
    }  
    
    float min_value = 1000000; // 1km范围内
    double ans      = 100000;
    int loc_num = route_data_.size() + 1;
    int min_start_num = previous_section_decide_num - 100;
    int max_end_num = previous_section_decide_num + 100;
    if(min_start_num <= search_minstart_num)
    {
      min_start_num = search_minstart_num;
    }
    if(max_end_num >= search_maxend_num)
    {
      max_end_num = search_maxend_num;
    }

    for (int i = min_start_num; i <= max_end_num; i++)
    {
      ans = pointDistanceSquare(rpt, route_data_[i]);

      if (ans == 0)
      {
        min_value = ans;
        loc_num   = i;
      }

      if (ans < min_value)
      {
        min_value = ans;
        loc_num   = i;
      }
    }  
    section_decide_num = loc_num; 
    previous_section_decide_num = section_decide_num;


    if(strange_points.size() == 0)
    {
      route_section_index_.push_back(0);
      route_section_index_.push_back(route_data_.size()-1);
      for(int j=0;j < route_data_.size();j++)
      {
        route_section_data_.push_back(route_data_[j]);
      }
    }
    else
    {
      for(int i=0;i <= strange_points.size(); i++)
      {
        if(i == 0)
        {
          if(section_decide_num < (strange_points[i] - 1))
          {
            for(int j=0;j < (strange_points[i]+1);j++)
            {
              route_section_data_.push_back(route_data_[j]);
            }
            route_section_index_.push_back(0);
            route_section_index_.push_back(strange_points[i]);
          }
        }
        else if(i == strange_points.size())
        {
          if(section_decide_num >= (strange_points[i-1] - 1))
          {
            for(int j=strange_points[i-1]+1;j < (route_data_.size());j++)
            {
              route_section_data_.push_back(route_data_[j]);
            }
            route_section_index_.push_back(strange_points[i-1]+1);
            route_section_index_.push_back(route_data_.size()-1);
          }
        }
        else
        {
          if(section_decide_num >= (strange_points[i-1] - 1) && section_decide_num < (strange_points[i] - 1))
          {
            for(int j=strange_points[i-1]+1;j < (strange_points[i]+1);j++)
            {
              route_section_data_.push_back(route_data_[j]);
            }
            route_section_index_.push_back(strange_points[i-1]+1);
            route_section_index_.push_back(strange_points[i]);
          }
        }
      } 
    }
  }
  else//与上一帧路径不同
  {
    route_section_data_.clear();
    vector< positionConf >().swap(route_section_data_);
    route_section_index_.clear();
    vector< int >().swap(route_section_index_);
 
    float min_value = 1000000; // 1km范围内
    double ans      = 100000;
    int loc_num = route_data_.size() + 1;
    for (int i = 0; i < route_data_.size(); i++)
    {
      ans = pointDistanceSquare(rpt, route_data_[i]);

      if (ans == 0)
      {
        min_value = ans;
        loc_num   = i;
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
    section_decide_num = loc_num; 
    previous_section_decide_num = section_decide_num;

    if(strange_points.size() == 0)
    {
      route_section_index_.push_back(0);
      route_section_index_.push_back(route_data_.size()-1);
      for(int j=0;j < route_data_.size();j++)
      {
        route_section_data_.push_back(route_data_[j]);
      }
    }
    else
    {
      for(int i=0;i <= strange_points.size(); i++)
      {
        if(i == 0)
        {
          if(section_decide_num < (strange_points[i] - 1))
          {
            for(int j=0;j < (strange_points[i]+1);j++)
            {
              route_section_data_.push_back(route_data_[j]);
            }
            route_section_index_.push_back(0);
            route_section_index_.push_back(strange_points[i]);
          }
        }
        else if(i == strange_points.size())
        {
          if(section_decide_num >= (strange_points[i-1] - 1))
          {
            for(int j=strange_points[i-1]+1;j < (route_data_.size());j++)
            {
              route_section_data_.push_back(route_data_[j]);
            }
            route_section_index_.push_back(strange_points[i-1]+1);
            route_section_index_.push_back(route_data_.size()-1);
          }
        }
        else
        {
          if(section_decide_num >= (strange_points[i-1] - 1) && section_decide_num < (strange_points[i] - 1))
          {
            for(int j=strange_points[i-1]+1;j < (strange_points[i]+1);j++)
            {
              route_section_data_.push_back(route_data_[j]);
            }
            route_section_index_.push_back(strange_points[i-1]+1);
            route_section_index_.push_back(strange_points[i]);
          }
        }
      }  
    }
  }
}

int LFControl::findClosestRefPoint(const positionConf &rpt,const int &previous_loc_num)
{
  float min_value = 1000000; // 1km范围内
  double ans      = 100000;
  int loc_num     = route_data_.size() + 1;

  if(previous_route_data_.size() == route_data_.size())
  {
    if(previous_route_data_[0].x == route_data_[0].x && previous_route_data_[0].y == route_data_[0].y && previous_route_data_[previous_route_data_.size()-1].x == route_data_[route_data_.size()-1].x && previous_route_data_[previous_route_data_.size()-1].y == route_data_[route_data_.size()-1].y)//与上一帧路径相同
    {
      int min_start_num = previous_loc_num - 100;
      int max_end_num = previous_loc_num + 100;
      if(min_start_num <= 0)
      {
        min_start_num = 0;
      }
      if(max_end_num >= route_data_.size())
      {
        max_end_num = route_data_.size();
      }

      for (int i = min_start_num; i < max_end_num; i++)
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
    }
    else//与上一帧路径不同
    {
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
    }
  }
  else//与上一帧路径不同
  {
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
  }  

  if (loc_num > route_data_.size())
  {
    ROS_INFO("Car is out of this map!!!\n");
    ROS_INFO("shutting down!\n");
    ros::shutdown();
  }     

  return loc_num;
}

int LFControl::findClosestFCRefPoint(const positionConf &rpt,const int &ref_loc_num)
{
  float min_value = 1000000; // 1km范围内
  double ans      = 100000;
  int loc_num     = ref_loc_num;

  int max_end_num = ref_loc_num + 400;
  
  if(max_end_num >= route_section_index_[1]+1)
  {
    max_end_num = route_section_index_[1]+1;
  }

  for (int i = ref_loc_num; i < max_end_num; i++)
  {
    ans = pointDistanceSquare(rpt, route_data_[i]);

    if (ans == 0)
    {
      min_value = ans;
      loc_num   = i;
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

void LFControl::recvHuacePosCallback(const location_sensor_msgs::IMUAndGNSSInfo &msg)
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

double LFControl::sumSquares3D(const double &in_x, const double &in_y, const double &in_z)
{
  return (in_x * in_x + in_y * in_y + in_z * in_z);
}

void LFControl::recvFusionPosCallback(const location_msgs::FusionDataInfo &msg)
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

void LFControl::recvPrescanRealtimePosCallback(const nav_msgs::Odometry &msg)
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
}

void LFControl::recvLocalPathCallback(const plan_msgs::DecisionInfo &msg)
{
  positionConf read_position = {0};
  lattice_route_data_.clear();
  vector< positionConf >().swap(lattice_route_data_);
  route_data_.clear();
  vector< positionConf >().swap(route_data_);

  if(msg.path_data_REF.size() > 0)
  {
	  for (int i = 0; i < msg.path_data_REF.size(); i++)
	  {
	    read_position.x        = msg.path_data_REF[i].x;
	    read_position.y        = msg.path_data_REF[i].y;
	    read_position.heading  = msg.path_data_REF[i].theta;
	    read_position.velocity = msg.path_data_REF[i].v;

	    lattice_route_data_.push_back(read_position);
	  }
	  plan_flag = msg.path_plan_valid;
          path_mode = msg.path_mode;

	  //将传来的路径点插值变密
	  int s = lattice_route_data_.size();
	  for (int i = 0; i < (s - 1); i++)
	  {
	    route_data_.push_back(lattice_route_data_[i]);

	    double l = sqrt((lattice_route_data_[i].x - lattice_route_data_[i + 1].x) *
		                (lattice_route_data_[i].x - lattice_route_data_[i + 1].x) +
		            (lattice_route_data_[i].y - lattice_route_data_[i + 1].y) *
		                (lattice_route_data_[i].y - lattice_route_data_[i + 1].y));
	    int n            = ceil(l / equal_length);
	    double dx        = (lattice_route_data_[i + 1].x - lattice_route_data_[i].x) / n;
	    double dy        = (lattice_route_data_[i + 1].y - lattice_route_data_[i].y) / n;
	    double dheading  = (lattice_route_data_[i + 1].heading - lattice_route_data_[i].heading) / n;
            double dheading_j = computedheading(lattice_route_data_[i].heading,lattice_route_data_[i + 1].heading);
	    double dvelocity = (lattice_route_data_[i + 1].velocity - lattice_route_data_[i].velocity) / n;
            if(abs(dheading_j) >= 90)
            {
              for (int j = 0; j < (n - 1); j++)
	      {
	        read_position.x        = lattice_route_data_[i].x + j * dx;
	        read_position.y        = lattice_route_data_[i].y + j * dy;

                read_position.heading  = lattice_route_data_[i+1].heading;
	        read_position.velocity = lattice_route_data_[i].velocity + j * dvelocity;
                if(read_position.velocity < tp_speed && read_position.velocity > 0)
                {
                  read_position.velocity = tp_speed;
                }
	        route_data_.push_back(read_position);
	      }
            }
            else
            {
              for (int j = 0; j < (n - 1); j++)
	      {
	        read_position.x        = lattice_route_data_[i].x + j * dx;
	        read_position.y        = lattice_route_data_[i].y + j * dy;

                if(n*abs(dheading) >= 180)
                {
                  if(dheading > 0)
                  {
                    dheading = (360 - lattice_route_data_[i + 1].heading + lattice_route_data_[i].heading)/n;
                    read_position.heading  = lattice_route_data_[i].heading - j * dheading;
                    if(read_position.heading < 0)
                    {
                      read_position.heading = read_position.heading + 360;
                    }
                  }
                  else//dheading < 0
                  {
                    dheading = (360 - lattice_route_data_[i].heading + lattice_route_data_[i+1].heading)/n;
                    read_position.heading  = lattice_route_data_[i].heading + j * dheading;
                    if(read_position.heading > 360)
                    {
                      read_position.heading = read_position.heading - 360;
                    }
                  } 
                }
                else
                {
                  read_position.heading  = lattice_route_data_[i].heading + j * dheading;
                }
	      
	        read_position.velocity = lattice_route_data_[i].velocity + j * dvelocity;
                if(read_position.velocity < tp_speed && read_position.velocity > 0)
                {
                  read_position.velocity = tp_speed;
                }
	        route_data_.push_back(read_position);
	      }
            }
	    
	  }
	  route_data_.push_back(lattice_route_data_[s - 1]);

	  math_tip_ = 2;
  }
  else
  {
    math_tip_ = 1;
  }
  
}

void LFControl::recvReflineStopCallback(const control_msgs::ADControlAGV &msg)
{
  refline_stop_msg = msg;
}

} // namespace control
