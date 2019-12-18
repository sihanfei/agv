#include "agv2p2.h"

namespace superg_agv
{
namespace drivers
{

AGV2P2::AGV2P2()
{

  ros::NodeHandle nh;

  agvstatus_flag = 0;
  agvwheelspd_flag = 0;
  turnleft_positive = 1;

  // agvinfo_sub = nh.subscribe("/drivers/com2agv/agv_status",10,&AGV2P2::recvAGVInfoCallback,this);
  // agvwheelspd_sub = nh.subscribe("/drivers/com2agv/agv_status2",10,&AGV2P2::recvAGVWheelSpdCallback,this);

  agv2p2info_pub = nh.advertise<control_msgs::AGV2P2Info>("/drivers/com2agv/agc2p2info", 10);
  // AGV2P2::agv2p2();

}

AGV2P2::~AGV2P2()
{

}

//主循环函数
void AGV2P2::agv2p2()
{
  ros::Rate loop_rate(100);

  while(ros::ok())
  {

    if(agvstatus_flag == 1 && agvwheelspd_flag == 1)
    {
      //格式
      coordinate_format = 1;

      //档位
      shift_compute(agvstatus,shift);

      //速度
      velocity_compute(agvstatus,agvwheelspd);

      velocityxy_compute(agvstatus,velocity_x,velocity_y);

      //角速率
      turnv_compute(agvstatus,turnv);
     

      agvstatus_flag = 0;
    }//end if flag

    ros::spinOnce();
    loop_rate.sleep();
  }//end while rosok
}

void AGV2P2::agv2p2_(control_msgs::AGVStatus2 &msg)
{
  coordinate_format = 1;

  shift_compute(agvstatus,shift);

  velocity_compute(agvstatus,agvwheelspd);
  // ROS_INFO_STREAM("velocity_compute----compute_ActualSpd:" << compute_ActualSpd << ",_agvwheelspd.velocity_c:" << agvwheelspd.velocity_c);
  velocityxy_compute(agvstatus,velocity_x,velocity_y);
  // ROS_INFO_STREAM("velocityxy_compute----compute_ActualSpd:" << compute_ActualSpd << ",_agvwheelspd.velocity_c:" << agvwheelspd.velocity_c);

  turnv_compute(agvstatus,turnv);

  msg.velocity_x        = velocity_x;
  msg.velocity_y        = velocity_y;
  msg.turnv             = turnv;
  msg.shift             = shift;
  msg.coordinate_format = coordinate_format;

  msg.compute_ActualSpd = compute_ActualSpd;
  
  msg.ActualSpd         = agvstatus.ActualSpd;
  msg.ActualAgl_R       = agvstatus.ActualAgl_R;
  msg.ActualAgl_F       = agvstatus.ActualAgl_F;
  msg.VEHMode           = agvstatus.VEHMode;
  msg.VEHFlt            = agvstatus.VEHFlt;
  msg.LiftStatus        = agvstatus.LiftStatus;
  msg.HVStatus          = agvstatus.HVStatus;
  msg.EStopStatus       = agvstatus.EStopStatus;
  msg.EPBStatus         = agvstatus.EPBStatus;
  msg.Dir_PRND          = agvstatus.Dir_PRND;
  msg.SOC               = agvstatus.SOC;
  msg.Rolling_Counter   = agvstatus.Rolling_Counter;
  // ROS_INFO_STREAM("agv2p2_----compute_ActualSpd:" << compute_ActualSpd << ",_agvwheelspd.velocity_c:" << agvwheelspd.velocity_c);
  // agv2p2info_pub.publish(AGV2P2Info);
}

//运算函数
void AGV2P2::shift_compute(const control_msgs::AGVStatus &_agvstatus,int &_shift)
{
      if(_agvstatus.Dir_PRND == 1)
      {
        _shift = 1;
      }
      else if(_agvstatus.Dir_PRND == 4)
      {
        _shift = 2;
      }
      else if(_agvstatus.Dir_PRND == 3)
      {
        _shift = 0;
      }
      else
      {
        _shift = 3;
      }
}

void AGV2P2::velocity_compute(control_msgs::AGVStatus &_agvstatus,const AgvWheelSpd &_agvwheelspd)
{
  if(_agvstatus.ActualAgl_F < 0.001 && _agvstatus.ActualAgl_F > -0.001)
  {
    _agvstatus.ActualAgl_F = 0;
  }
  if(_agvstatus.ActualAgl_R < 0.001 && _agvstatus.ActualAgl_R > -0.001)
  {
    _agvstatus.ActualAgl_R = 0;
  }
  if(_agvstatus.ActualAgl_F == 0)
  {
    if(_agvstatus.ActualAgl_R == 0)//前后轮转角皆为0
    {
      _dphi = 0;
      R_turn_f = 0;
      R_turn_r = 0;
      R = 0;
      // _agvstatus.ActualSpd = _agvwheelspd.velocity_c;
      compute_ActualSpd = _agvwheelspd.velocity_c;
    }
    else//前轮转角为0 后轮转角不为0
    {
      if(turnleft_positive == 0)//左转为正
      {
        _dphi = atan(tan(-_agvstatus.ActualAgl_R*pi/180)/2);
        R = (_agvstatus.ActualAgl_R/fabs(_agvstatus.ActualAgl_R))*(5.5085 + 5.5085)*sqrt(0.25 + 1/(tan(_agvstatus.ActualAgl_R*pi/180)*tan(_agvstatus.ActualAgl_R*pi/180)));
        R_turn_f = sqrt((R)*(R) - (5.5085)*(5.5085));
        R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
        // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
        compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
      }
      else//右转为正
      {
        _dphi = atan(tan(_agvstatus.ActualAgl_R*pi/180)/2);
        R = -(_agvstatus.ActualAgl_R/fabs(_agvstatus.ActualAgl_R))*(5.5085 + 5.5085)*sqrt(0.25 + 1/(tan(_agvstatus.ActualAgl_R*pi/180)*tan(_agvstatus.ActualAgl_R*pi/180)));
        R_turn_f = sqrt((R)*(R) - (5.5085)*(5.5085));
        R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
        // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
        compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
      }
    }
  }
  else
  {
    if(_agvstatus.ActualAgl_R == 0)//前轮转角不为0 后轮转角为0
    {
      if(turnleft_positive == 0)//左转为正
      {
        _dphi = atan(tan(-_agvstatus.ActualAgl_F*pi/180)/2);
        R = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)*sqrt(0.25 + 1/(tan(_agvstatus.ActualAgl_F*pi/180)*tan(_agvstatus.ActualAgl_F*pi/180)));
        R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
        R_turn_r = sqrt(R*R - (5.5085)*(5.5085));
        // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
        compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
      }
      else//右转为正
      {
        _dphi = atan(tan(_agvstatus.ActualAgl_F*pi/180)/2);
        R = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)*sqrt(0.25 + 1/(tan(_agvstatus.ActualAgl_F*pi/180)*tan(_agvstatus.ActualAgl_F*pi/180)));
        R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
        R_turn_r = sqrt(R*R - (5.5085)*(5.5085));
        // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
        compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
      }
    }
    else//前后轮转角皆不为0
    {
      if(_agvstatus.ActualAgl_F*_agvstatus.ActualAgl_R < 0)//反向转向
      {
        if(fabs(_agvstatus.ActualAgl_F) > fabs(_agvstatus.ActualAgl_R))//前轮转角大于后轮转角
        {
          if(turnleft_positive == 0)//左转为正
          {
            _dphi = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan(fabs(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))/2);
            R = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)/2*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
          else//右转为正
          {
            _dphi = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan(fabs(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))/2);
            R = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)/2*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
        }
        else if(fabs(_agvstatus.ActualAgl_F) < fabs(_agvstatus.ActualAgl_R))//前轮转角小于后轮转角
        {
          if(turnleft_positive == 0)//左转为正
          {
            _dphi = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan(fabs(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))/2);
            R = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)/2*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
          else//右转为正
          {
            _dphi = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan(fabs(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))/2);
            R = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)/2*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
        }
        else//前后轮转角大小相等
        {
          if(turnleft_positive == 0)//左转为正
          {
            _dphi = 0;
            R = (5.5085 + 5.5085)*0.5/tan(-_agvstatus.ActualAgl_F*pi/180);
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
          else//右转为正
          {
            _dphi = 0;
            R = (5.5085 + 5.5085)*0.5/tan(_agvstatus.ActualAgl_F*pi/180);
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
        }
      }
      else//同向转向
      {
        if(fabs(_agvstatus.ActualAgl_F) > fabs(_agvstatus.ActualAgl_R))//前轮转角大于后轮转角
        {
          if(turnleft_positive == 0)//左转为正
          {
            _dphi = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))/2);
            R = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)*0.5*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
          else//右转为正
          {
            _dphi = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))/2);
            R = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)*0.5*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
        }
        else if(fabs(_agvstatus.ActualAgl_F) < fabs(_agvstatus.ActualAgl_R))//前轮转角小于后轮转角
        {
          if(turnleft_positive == 0)//左转为正
          {
            _dphi = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))/2);
            R = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)*0.5*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
          else//右转为正
          {
            _dphi = (_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*atan((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))/2);
            R = -(_agvstatus.ActualAgl_F/fabs(_agvstatus.ActualAgl_F))*(5.5085 + 5.5085)*0.5*sqrt((4 + (tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))));
            R_turn_f = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 - fabs(_dphi)));
            R_turn_r = sqrt(R*R + (5.5085)*(5.5085) - 2*fabs(R)*5.5085*cos(pi/2 + fabs(_dphi)));
            // _agvstatus.ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
            compute_ActualSpd = (fabs((_agvwheelspd.fc_s)/R_turn_f) + fabs((_agvwheelspd.rc_s)/R_turn_r))/2.0*fabs(R);
          }
        }
        else//前后轮转角大小相等
        {
          if(turnleft_positive == 0)//左转为正
          {
            _dphi = -_agvstatus.ActualAgl_F*pi/180;
            R = 0;
            R_turn_f = 0;
            R_turn_r = 0;
            // _agvstatus.ActualSpd = _agvwheelspd.velocity_c;
            compute_ActualSpd = _agvwheelspd.velocity_c;
          }
          else//右转为正
          {
            _dphi = _agvstatus.ActualAgl_F*pi/180;
            R = 0;
            R_turn_f = 0;
            R_turn_r = 0;
            // _agvstatus.ActualSpd = _agvwheelspd.velocity_c;
            compute_ActualSpd = _agvwheelspd.velocity_c;
          }
        }
      }
    }
  }
}

void AGV2P2::velocityxy_compute(const control_msgs::AGVStatus &_agvstatus,double &_velocity_x,double &_velocity_y)
{
  if(_agvstatus.Dir_PRND == 1)
  {
    if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_F == 0)
    {
      _velocity_x = 0;
      // _velocity_y = _agvstatus.ActualSpd;
      _velocity_y = compute_ActualSpd;
    }
    else
    {
      if(_agvstatus.ActualAgl_R*_agvstatus.ActualAgl_F >= 0)//同向转向
      {
        if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//同向斜行
        {
          // _velocity_x = _agvstatus.ActualSpd*sin(_agvstatus.ActualAgl_F*pi/180);
          // _velocity_y = _agvstatus.ActualSpd*cos(_agvstatus.ActualAgl_F*pi/180);
          _velocity_x = compute_ActualSpd*sin(_agvstatus.ActualAgl_F*pi/180);
          _velocity_y = compute_ActualSpd*cos(_agvstatus.ActualAgl_F*pi/180);
        }
        else//同向非斜行
        {
          // _velocity_x = _agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          // _velocity_y = _agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_x = compute_ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = compute_ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
      else//反向转向
      {
        if(fabs(_agvstatus.ActualAgl_R) == fabs(_agvstatus.ActualAgl_F))
        {
          _velocity_x = 0;
          // _velocity_y = _agvstatus.ActualSpd;
          _velocity_y = compute_ActualSpd;
        }
        else
        {
          // _velocity_x = _agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          // _velocity_y = _agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_x = compute_ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = compute_ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
    }
  }
  else if(_agvstatus.Dir_PRND == 4)
  {
    if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_F == 0)
    {
      _velocity_x = 0;
      // _velocity_y = -_agvstatus.ActualSpd;
      _velocity_y = -compute_ActualSpd;
    }
    else
    {
      if(_agvstatus.ActualAgl_R*_agvstatus.ActualAgl_F >= 0)//同向转向
      {
        if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//同向斜行
        {
          // _velocity_x = -_agvstatus.ActualSpd*sin(_agvstatus.ActualAgl_F*pi/180);
          // _velocity_y = -_agvstatus.ActualSpd*cos(_agvstatus.ActualAgl_F*pi/180);
          _velocity_x = -compute_ActualSpd*sin(_agvstatus.ActualAgl_F*pi/180);
          _velocity_y = -compute_ActualSpd*cos(_agvstatus.ActualAgl_F*pi/180); 
        }
        else//同向非斜行
        {
          // _velocity_x = -_agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          // _velocity_y = -_agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_x = -compute_ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = -compute_ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
      else//反向转向
      {
        if(fabs(_agvstatus.ActualAgl_R) == fabs(_agvstatus.ActualAgl_F))
        {
          _velocity_x = 0;
          // _velocity_y = -_agvstatus.ActualSpd;
          _velocity_y = -compute_ActualSpd;
        }
        else
        {
          // _velocity_x = -_agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          // _velocity_y = -_agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_x = -compute_ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = -compute_ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
    }
  }
  else
  {
    _velocity_x = 0;
    _velocity_y = 0;
  }
  _velocity_x = _velocity_x*3.6;
  _velocity_y = _velocity_y*3.6;
}

void AGV2P2::turnv_compute(const control_msgs::AGVStatus &_agvstatus,double &_turnv)
{
  if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_R == 0)
  {
    _turnv = 0;
  }
  else
  {
    if(_agvstatus.ActualAgl_R*_agvstatus.ActualAgl_F >= 0)//同向转向
    {
      if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//同向斜行
      {
        _turnv = 0;
      }
      else//同向非斜行
      {
        _turnv = (180/pi)*_agvstatus.ActualSpd/(5.5085*sqrt((4+(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) + tan(fabs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) - tan(fabs(_agvstatus.ActualAgl_F)*pi/180)))));
      }
    }
    else//反向转向
    {
      _turnv = (180/pi)*_agvstatus.ActualSpd/(5.5085*sqrt((4+(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_F)*pi/180) - tan(fabs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180))*(tan(fabs(_agvstatus.ActualAgl_R)*pi/180) + tan(fabs(_agvstatus.ActualAgl_F)*pi/180)))));
    }
  }
}

void AGV2P2::setAgvstatus(const control_msgs::AGVStatus &msg)
{
  agvstatus = msg;
  
}

void AGV2P2::setAgvwheelspd(const uint8_t rec_buf[],control_msgs::AGVStatus2 &msg)
{

  // for(int i = 0;i<8;++i)
  // {
  //   ROS_INFO_STREAM("setAgvstatus_" << i << ":" << rec_buf[i]);
  // }

  agvwheelspd.fl_s		= msg.WheelRotSpd_FL/25.8*2*pi/60*0.673;
  agvwheelspd.fr_s		= msg.WheelRotSpd_FR/25.8*2*pi/60*0.673;
  agvwheelspd.rl_s		= msg.WheelRotSpd_RL/25.8*2*pi/60*0.673;
  agvwheelspd.rr_s		= msg.WheelRotSpd_RR/25.8*2*pi/60*0.673;

  agvwheelspd.fc_s = (agvwheelspd.fl_s + agvwheelspd.fr_s)/2.0;
  agvwheelspd.rc_s = (agvwheelspd.rl_s + agvwheelspd.rr_s)/2.0;

  agvwheelspd.velocity_c = fabs((agvwheelspd.fc_s + agvwheelspd.rc_s)/2.0);

  // ROS_INFO_STREAM("msg.WheelRotSpd_FL:" << msg.WheelRotSpd_FL << ",msg.WheelRotSpd_FR:" << msg.WheelRotSpd_FR << ",msg.WheelRotSpd_RL:" << msg.WheelRotSpd_RL << ",msg.WheelRotSpd_RR:" << msg.WheelRotSpd_RR);
  // ROS_INFO_STREAM("setAgvwheelspd----compute_ActualSpd:" << compute_ActualSpd << ",_agvwheelspd.velocity_c:" << agvwheelspd.velocity_c);

  // agvwheelspd.fl_s		= ((rec_buf[1]<<8 | rec_buf[0]) - 12000)/25.8*2*pi/60*0.673;
  // agvwheelspd.fr_s		= ((rec_buf[3]<<8 | rec_buf[2]) - 12000)/25.8*2*pi/60*0.673;
  // agvwheelspd.rl_s		= ((rec_buf[5]<<8 | rec_buf[4]) - 12000)/25.8*2*pi/60*0.673;
  // agvwheelspd.rr_s		= ((rec_buf[7]<<8 | rec_buf[6]) - 12000)/25.8*2*pi/60*0.673;

  // agvwheelspd.fc_s = (agvwheelspd.fl_s + agvwheelspd.fr_s)/2.0;
  // agvwheelspd.rc_s = (agvwheelspd.rl_s + agvwheelspd.rr_s)/2.0;

  // agvwheelspd.velocity_c = fabs((agvwheelspd.fc_s + agvwheelspd.rc_s)/2.0);
}

//回调函数
void AGV2P2::recvAGVInfoCallback(const control_msgs::AGVStatus &msg)
{
  agvstatus = msg;

  agvstatus_flag = 1;
}

void AGV2P2::recvAGVWheelSpdCallback(const control_msgs::AGVStatus2 &msg)
{
  agvwheelspd.fl_s = msg.WheelRotSpd_FL/25.8*2*pi/60*0.673;
  agvwheelspd.fr_s = msg.WheelRotSpd_FR/25.8*2*pi/60*0.673;
  agvwheelspd.rl_s = msg.WheelRotSpd_RL/25.8*2*pi/60*0.673;
  agvwheelspd.rr_s = msg.WheelRotSpd_RR/25.8*2*pi/60*0.673;

  agvwheelspd.fc_s = (agvwheelspd.fl_s + agvwheelspd.fr_s)/2.0;
  agvwheelspd.rc_s = (agvwheelspd.rl_s + agvwheelspd.rr_s)/2.0;

  agvwheelspd.velocity_c = fabs((agvwheelspd.fc_s + agvwheelspd.rc_s)/2.0);

  agvwheelspd_flag = 1;
}

}
}
