#include "agv2p2.h"

namespace superg_agv
{
namespace drivers
{

AGV2P2::AGV2P2(ros::NodeHandle &nh)
{

  agvstatus_flag = 0;

  agvinfo_sub = nh.subscribe("/drivers/com2agv/agv_status2",10,&AGV2P2::recvAGVInfoCallback,this);

  // AGV2P2::agv2p2();

}

AGV2P2::~AGV2P2()
{

}

//主循环函数
void AGV2P2::agv2p2(int &ch_, int &dev_)
{
  ros::Rate loop_rate(100);

  while(ros::ok())
  {

    // if(agvstatus_flag == 1)
    // {
    //   //格式
    //   coordinate_format = 1;

    //   //档位
    //   shift_compute(agvstatus,shift);

    //   //角速率
    //   turnv_compute(agvstatus,turnv);
      
    //   //速度
    //   velocity_compute(agvstatus,velocity_x,velocity_y);

    //   //将参数压入buf
      // arg2canbuf();

    //   //向can写数据
    //   writeCANtoP2(dev_,canid,canbuf,canbuf_length);

    //   agvstatus_flag = 0;
    // }//end if flag

    // canbuf[0] = 0x01;
    // canbuf[1] = 0x02;
    // canbuf[2] = 0x03;
    // canbuf[3] = 0x04;
    // canbuf[4] = 0x05;
    // canbuf[5] = 0x06;
    // canbuf[6] = 0x07;
    // canbuf[7] = 0x08;
    // writeCANtoP2(dev_,canid,canbuf,8);
    // sleep(1);

    ros::spinOnce();

    if(1 == agvstatus_flag)
    {

      arg2canbuf();

      writeCANtoP2(dev_,canid,canbuf,canbuf_length);

      agvstatus_flag = 0;

    }

    loop_rate.sleep();
  }//end while rosok
}

//运算函数
void AGV2P2::shift_compute(const control_msgs::AGVStatus2 &_agvstatus,int &_shift)
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

void AGV2P2::turnv_compute(const control_msgs::AGVStatus2 &_agvstatus,double &_turnv)
{
  if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_F == 0)
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
        _turnv = (180/pi)*_agvstatus.ActualSpd/(5.5085*sqrt((4+(tan(abs(_agvstatus.ActualAgl_F)*pi/180) + tan(abs(_agvstatus.ActualAgl_R)*pi/180))*(tan(abs(_agvstatus.ActualAgl_F)*pi/180) + tan(abs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(abs(_agvstatus.ActualAgl_R)*pi/180) - tan(abs(_agvstatus.ActualAgl_F)*pi/180))*(tan(abs(_agvstatus.ActualAgl_R)*pi/180) - tan(abs(_agvstatus.ActualAgl_F)*pi/180)))));
      }
    }
    else//反向转向
    {
      _turnv = (180/pi)*_agvstatus.ActualSpd/(5.5085*sqrt((4+(tan(abs(_agvstatus.ActualAgl_F)*pi/180) - tan(abs(_agvstatus.ActualAgl_R)*pi/180))*(tan(abs(_agvstatus.ActualAgl_F)*pi/180) - tan(abs(_agvstatus.ActualAgl_R)*pi/180)))/((tan(abs(_agvstatus.ActualAgl_R)*pi/180) + tan(abs(_agvstatus.ActualAgl_F)*pi/180))*(tan(abs(_agvstatus.ActualAgl_R)*pi/180) + tan(abs(_agvstatus.ActualAgl_F)*pi/180)))));
    }
  }
}
    
void AGV2P2::velocity_compute(const control_msgs::AGVStatus2 &_agvstatus,double &_velocity_x,double &_velocity_y)
{
  if(_agvstatus.Dir_PRND == 1)
  {
    if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_F == 0)
    {
      _velocity_x = 0;
      _velocity_y = _agvstatus.ActualSpd;
    }
    else
    {
      if(_agvstatus.ActualAgl_R*_agvstatus.ActualAgl_F >= 0)//同向转向
      {
        if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//同向斜行
        {
          _velocity_x = _agvstatus.ActualSpd*sin(_agvstatus.ActualAgl_F*pi/180);
          _velocity_y = _agvstatus.ActualSpd*cos(_agvstatus.ActualAgl_F*pi/180); 
        }
        else//同向非斜行
        {
          _velocity_x = _agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = _agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
      else//反向转向
      {
        if(abs(_agvstatus.ActualAgl_R) == abs(_agvstatus.ActualAgl_F))
        {
          _velocity_x = 0;
          _velocity_y = _agvstatus.ActualSpd;
        }
        else
        {
          _velocity_x = _agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = _agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
    }
  }
  else if(_agvstatus.Dir_PRND == 4)
  {
    if(_agvstatus.ActualAgl_R == 0 && _agvstatus.ActualAgl_F == 0)
    {
      _velocity_x = 0;
      _velocity_y = -_agvstatus.ActualSpd;
    }
    else
    {
      if(_agvstatus.ActualAgl_R*_agvstatus.ActualAgl_F >= 0)//同向转向
      {
        if(_agvstatus.ActualAgl_R == _agvstatus.ActualAgl_F)//同向斜行
        {
          _velocity_x = -_agvstatus.ActualSpd*sin(_agvstatus.ActualAgl_F*pi/180);
          _velocity_y = -_agvstatus.ActualSpd*cos(_agvstatus.ActualAgl_F*pi/180); 
        }
        else//同向非斜行
        {
          _velocity_x = -_agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = -_agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
        }
      }
      else//反向转向
      {
        if(abs(_agvstatus.ActualAgl_R) == abs(_agvstatus.ActualAgl_F))
        {
          _velocity_x = 0;
          _velocity_y = -_agvstatus.ActualSpd;
        }
        else
        {
          _velocity_x = -_agvstatus.ActualSpd*sin(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
          _velocity_y = -_agvstatus.ActualSpd*cos(atan((tan(_agvstatus.ActualAgl_F*pi/180) + tan(_agvstatus.ActualAgl_R*pi/180))/2));
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

//参数压入canbuf中
void AGV2P2::arg2canbuf()
{
  printf("arg:%f,%f,%f\r\n",velocity_x,velocity_y,turnv);
  // int16_t toint16 = int16_t(velocity_x * 100);
  // canbuf[0] = uint8_t(toint16 >> 8);
  // canbuf[1] = uint8_t(toint16);
  // toint16 = int16_t(velocity_y * 100);
  // canbuf[2] = uint8_t(toint16 >> 8);
  // canbuf[3] = uint8_t(toint16);
  // toint16 = int16_t(turnv * 100);
  // canbuf[4] = uint8_t(toint16 >> 8);
  // canbuf[5] = uint8_t(toint16);
  // canbuf[6] = uint8_t(shift);
  // canbuf[7] = uint8_t(0);
  int16_t toint16 = int16_t(agvstatus.velocity_x * 100);
  canbuf[0] = uint8_t(toint16 >> 8);
  canbuf[1] = uint8_t(toint16);
  toint16 = int16_t(agvstatus.velocity_y * 100);
  canbuf[2] = uint8_t(toint16 >> 8);
  canbuf[3] = uint8_t(toint16);
  toint16 = int16_t(agvstatus.turnv * 100);
  canbuf[4] = uint8_t(toint16 >> 8);
  canbuf[5] = uint8_t(toint16);
  canbuf[6] = uint8_t(agvstatus.shift);
  canbuf[7] = uint8_t(agvstatus.coordinate_format);
}

//回调函数
void AGV2P2::recvAGVInfoCallback(const control_msgs::AGVStatus2 &msg)
{
  agvstatus = msg;

  agvstatus_flag = 1;
}

}
}
