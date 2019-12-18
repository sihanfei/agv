#ifndef AGV2P2_H_
#define AGV2P2_H_

#include "ros/ros.h"

#include <iostream>
#include "math.h"
#include "can_rw.h"

#include <control_msgs/AGVStatus2.h>

#define pi 3.1415926

namespace superg_agv
{
namespace drivers
{
class AGV2P2 : public CANDrivers
{
  public:
    AGV2P2(ros::NodeHandle &nh);
    ~AGV2P2();

    //订阅agv轮速转角回调函数
    void recvAGVInfoCallback(const control_msgs::AGVStatus2 &msg);

    //主循环函数
    void agv2p2(int &ch_, int &dev_);
    
    //运算函数
    void shift_compute(const control_msgs::AGVStatus2 &_agvstatus,int &_shift);
    void turnv_compute(const control_msgs::AGVStatus2 &_agvstatus,double &_turnv);
    void velocity_compute(const control_msgs::AGVStatus2 &_agvstatus,double &_velocity_x,double &_velocity_y);
    void arg2canbuf();

  protected:

    //flags
    int agvstatus_flag;




    ///////////////////Variables need to be transmitted to P2 via CAN///////////////////////////////
    int coordinate_format;
    int shift;

    double velocity_x;//车辆坐标系下agv在x方向速度 单位:km/h
    double velocity_y;//车辆坐标系下agv在y方向速度 单位:km/h

    double turnv;//agv转动角速率 单位:deg/s
    ////////////////////////////////////////////////////////////////////////////////////////////////

    uint8_t canbuf[8];

    const uint32_t canid = 820;
    
    const uint8_t canbuf_length = 8;

    //AGV INFO 
    control_msgs::AGVStatus2 agvstatus;

    //subscribe and publish
    ros::Subscriber agvinfo_sub;

};
 
}
}

#endif
