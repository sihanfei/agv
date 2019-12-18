#include "tp_control.h"
#include "control.h"

using namespace std;
using namespace control;

namespace control
{

TPControl::TPControl(ros::NodeHandle &nh)
{
  TPControl::initialization(nh);
  
  TPControl::tpControl();
}

TPControl::~TPControl()
{
  ROS_INFO("Object TPControl deleted.");
}

void TPControl::initialization(ros::NodeHandle &nh)
{
  ROS_INFO("Start initialization...");

  decision_start_flag = 0;
  tinypark_success_flag = 0;
  agvstatus_start_flag = 0;
  dir_judge_flag = 0;
  init_dir = 0;

  ROS_INFO("Start load paramaters");
  nh.getParam("/control/mode",mode);
  nh.getParam("/control/tp_speed",tp_speed);
  nh.getParam("/control/park_err_tolerance",park_err_tolerance);
  ROS_INFO("Paramaters load finished.");
  
  if(mode == 0)
  {
    agv_status_sub = nh.subscribe("/drivers/com2agv/agv_status",10,&TPControl::recvPrescanAGVStatusCallback,this);
    decision_info_sub = nh.subscribe("/plan/decision_info",10,&TPControl::recvPrescanDecisionInfoCallback,this);
    control_vcu_pub = nh.advertise<geometry_msgs::PoseStamped>("/control/control_agv",BUF_LEN,true);
  }
  else
  {
    agv_status_sub = nh.subscribe("/drivers/com2agv/agv_status",10,&TPControl::recvAGVStatusCallback,this);
    decision_info_sub = nh.subscribe("/plan/decision_info",10,&TPControl::recvDecisionInfoCallback,this);
    control_vcu_pub = nh.advertise<control_msgs::ADControlAGV>("/control/control_agv",BUF_LEN,true);
  }

  ROS_INFO("Initialization finished.");
}

void TPControl::tpControl()
{
  ROS_INFO("Start main loop...");
  ros::Rate loop_rate(FRE);

  while(ros::ok())//while s
  {
    cout << "path mode: " << path_mode << " dir_judge_flag: " << dir_judge_flag << " init dir: " << init_dir << " success flag: " << tinypark_success_flag << endl;
    if(decision_start_flag == 1 && path_mode == 4 && dir_judge_flag == 1)//flag s
    {
      if(tinypark_success_flag == 1)//已经停车成功一直发送保持停止命令 //success s
      {
         if(mode == 0)
         {
           prescan_control_msg.pose.orientation.x = 0;
           prescan_control_msg.pose.orientation.y = 0;
           prescan_control_msg.pose.position.x = 0;
         }
         else
         {
           control_msg.VehAgl_F = 0;
           control_msg.VehAgl_R = 0;
           control_msg.Vel_Req = 0;
         }
      }
      else//停车还未成功发送调整控制命令
      {
        if(agv_status.ActualSpd <= 0.0001)
        {
          if(abs(dy) < park_err_tolerance)
          {
            if(mode == 0)
            {
              prescan_control_msg.pose.orientation.x = 0;
              prescan_control_msg.pose.orientation.y = 0;
              prescan_control_msg.pose.position.x = 0;
            }
            else
            {
              control_msg.VehAgl_F = 0;
              control_msg.VehAgl_R = 0;
              control_msg.Vel_Req = 0;
            }
            tinypark_success_flag = 1;
          }
        }
        else
        {
          if(init_dir == 1)//进入精确停车时处于前进档
          {
            if(dy < 0)//dy < 0
            {
              if(abs(dy) < park_err_tolerance)
              {
                if(mode == 0)
                {
                  prescan_control_msg.pose.orientation.x = 0;
                  prescan_control_msg.pose.orientation.y = 0;
                  prescan_control_msg.pose.position.x = 0;
                }
                else
                {
                  control_msg.VehAgl_F = 0;
                  control_msg.VehAgl_R = 0;
                  control_msg.Vel_Req = 0;
                }
              }
              else
              {
                if(mode == 0)
		{
		  prescan_control_msg.pose.orientation.x = 0;
		  prescan_control_msg.pose.orientation.y = 0;
		  prescan_control_msg.pose.position.x = tp_speed;
		}
		else
		{
		  control_msg.VehAgl_F = 0;
		  control_msg.VehAgl_R = 0;
		  control_msg.Vel_Req = tp_speed;
		}
              }
            }
            else//dy > 0
            {
              if(abs(dy) < park_err_tolerance)
              {
                if(mode == 0)
                {
                  prescan_control_msg.pose.orientation.x = 0;
                  prescan_control_msg.pose.orientation.y = 0;
                  prescan_control_msg.pose.position.x = 0;
                }
                else
                {
                  control_msg.VehAgl_F = 0;
                  control_msg.VehAgl_R = 0;
                  control_msg.Vel_Req = 0;
                }
              }
              else
              {
                if(mode == 0)
		{
		  prescan_control_msg.pose.orientation.x = 0;
		  prescan_control_msg.pose.orientation.y = 0;
		  prescan_control_msg.pose.position.x = -tp_speed;
		}
		else
		{
		  control_msg.VehAgl_F = 0;
		  control_msg.VehAgl_R = 0;
		  control_msg.Vel_Req = -tp_speed;
		}
              }
            }
          }
          else//进入精确停车时处于后退档
          {
            if(dy < 0)
            {
              if(abs(dy) < park_err_tolerance)
              {
                if(mode == 0)
                {
                  prescan_control_msg.pose.orientation.x = 0;
                  prescan_control_msg.pose.orientation.y = 0;
                  prescan_control_msg.pose.position.x = 0;
                }
                else
                {
                  control_msg.VehAgl_F = 0;
                  control_msg.VehAgl_R = 0;
                  control_msg.Vel_Req = 0;
                }
              }
              else
              {
                if(mode == 0)
		{
		  prescan_control_msg.pose.orientation.x = 0;
		  prescan_control_msg.pose.orientation.y = 0;
		  prescan_control_msg.pose.position.x = -tp_speed;
		}
		else
		{
		  control_msg.VehAgl_F = 0;
		  control_msg.VehAgl_R = 0;
		  control_msg.Vel_Req = -tp_speed;
		}
              }
            }
            else
            {
              if(abs(dy) < park_err_tolerance)
              {
                if(mode == 0)
                {
                  prescan_control_msg.pose.orientation.x = 0;
                  prescan_control_msg.pose.orientation.y = 0;
                  prescan_control_msg.pose.position.x = 0;
                }
                else
                {
                  control_msg.VehAgl_F = 0;
                  control_msg.VehAgl_R = 0;
                  control_msg.Vel_Req = 0;
                }
              }
              else
              {
                if(mode == 0)
		{
		  prescan_control_msg.pose.orientation.x = 0;
		  prescan_control_msg.pose.orientation.y = 0;
		  prescan_control_msg.pose.position.x = tp_speed;
		}
		else
		{
		  control_msg.VehAgl_F = 0;
		  control_msg.VehAgl_R = 0;
		  control_msg.Vel_Req = tp_speed;
		}
              }
            }
          }
        }
      }
      if(mode == 0)
      {
        control_vcu_pub.publish(prescan_control_msg);
      }
      else
      {
        control_vcu_pub.publish(control_msg);
      }

      decision_start_flag = 0;
      agvstatus_start_flag = 0;
    }//flag e
    
    ros::spinOnce();
    loop_rate.sleep();
  }//while e
}

void TPControl::recvDecisionInfoCallback(const plan_msgs::DecisionInfo &msg)
{
  dx = msg.relative_position.x;
  dy = msg.relative_position.y;
  dheading = msg.relative_position.z;
  path_mode = msg.path_mode;

  if(path_mode != 4)//脱离精准停车状态后停车成功标志置0 档位判断标志置0 初始档位置0
  {
    tinypark_success_flag = 0;
    dir_judge_flag = 0;
    init_dir = 0;
  }

  decision_start_flag = 1;
}

void TPControl::recvPrescanDecisionInfoCallback(const geometry_msgs::Pose &msg)
{
  dx = msg.position.x;
  dy = msg.position.y;
  dheading = msg.position.z;
  path_mode = msg.orientation.x;
  
  if(path_mode != 4)//脱离精准停车状态后停车成功标志置0 档位判断标志置0 初始档位置0
  {
    tinypark_success_flag = 0;
    dir_judge_flag = 0;
    init_dir = 0;
  }

  decision_start_flag = 1;
}

void TPControl::recvAGVStatusCallback(const control_msgs::AGVStatus &msg)
{
  agv_status.ActualSpd = msg.ActualSpd;
  agv_status.ActualAgl_R = msg.ActualAgl_R;
  agv_status.ActualAgl_F = msg.ActualAgl_F;
  agv_status.VEHMode = msg.VEHMode;
  agv_status.VEHFlt = msg.VEHFlt;
  agv_status.LiftStatus = msg.LiftStatus;
  agv_status.HVStatus = msg.HVStatus;
  agv_status.EStopStatus = msg.EStopStatus;
  agv_status.EPBStatus = msg.EPBStatus;
  agv_status.Dir_PRND = msg.Dir_PRND;
  agv_status.SOC = msg.SOC;
  agv_status.Rolling_Counter = msg.Rolling_Counter;

  if(dir_judge_flag == 0)
  {
    if(path_mode == 4)
    {
      init_dir = agv_status.Dir_PRND;
      dir_judge_flag = 1;
    }
  }

  agvstatus_start_flag = 1;
}

void TPControl::recvPrescanAGVStatusCallback(const nav_msgs::Odometry &msg)
{
  prescan_agv_status.ActualSpd = msg.pose.pose.position.x;
  prescan_agv_status.ActualAgl_R = msg.pose.pose.position.y;
  prescan_agv_status.ActualAgl_F = msg.pose.pose.position.z;
  prescan_agv_status.VEHMode = msg.pose.pose.orientation.x;
  prescan_agv_status.VEHFlt = msg.pose.pose.orientation.y;
  prescan_agv_status.LiftStatus = msg.pose.pose.orientation.z;
  prescan_agv_status.HVStatus = msg.pose.pose.orientation.w;
  prescan_agv_status.EStopStatus = msg.twist.twist.linear.x;
  prescan_agv_status.EPBStatus = msg.twist.twist.linear.y;
  prescan_agv_status.Dir_PRND = msg.twist.twist.linear.z;
  prescan_agv_status.SOC = msg.twist.twist.angular.x;
  prescan_agv_status.Rolling_Counter = msg.twist.twist.angular.y;

  if(dir_judge_flag == 0)
  {
    if(path_mode == 4)
    {
      init_dir = agv_status.Dir_PRND;
      dir_judge_flag = 1;
    }
  }

  agvstatus_start_flag = 1;
}
} //namespace control
