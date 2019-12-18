#ifndef AGV2P2_H_
#define AGV2P2_H_

#include "ros/ros.h"

#include <iostream>
#include "math.h"

#include <control_msgs/AGVStatus.h>
#include <control_msgs/AGVStatus2.h>
#include <control_msgs/AGV2P2Info.h>

#define pi 3.1415926

namespace superg_agv
{
namespace drivers
{
struct AgvWheelSpd
{
  double fl_s;
  double fr_s;
  double rl_s;
  double rr_s;

  double fc_s;
  double rc_s;

  double velocity_c;
};

class AGV2P2
{
  public:
    AGV2P2();
    ~AGV2P2();

    //订阅agv轮速转角回调函数
    void recvAGVInfoCallback(const control_msgs::AGVStatus &msg);
    void recvAGVWheelSpdCallback(const control_msgs::AGVStatus2 &msg);

    //主循环函数
    void agv2p2();
    
    //运算函数
    void shift_compute(const control_msgs::AGVStatus &_agvstatus,int &_shift);
    void velocity_compute(control_msgs::AGVStatus &_agvstatus,const AgvWheelSpd &_agvwheelspd);
    void velocityxy_compute(const control_msgs::AGVStatus &_agvstatus,double &_velocity_x,double &_velocity_y);
    void turnv_compute(const control_msgs::AGVStatus &_agvstatus,double &_turnv);
    void setAgvstatus(const control_msgs::AGVStatus &msg);
    void setAgvwheelspd(const uint8_t rec_buf[],control_msgs::AGVStatus2 &msg);
    void agv2p2_(control_msgs::AGVStatus2 &msg);

  protected:

    //flags
    int agvstatus_flag;
    int agvwheelspd_flag;
    int turnleft_positive;

    //variables 运算中用到的变量
    double _dphi;
    double R;
    double R_turn_f;
    double R_turn_r;

    ///////////////////Variables need to be transmitted to P2 via CAN///////////////////////////////
    int coordinate_format;
    int shift;

    double velocity_x;//车辆坐标系下agv在x方向速度 单位:km/h
    double velocity_y;//车辆坐标系下agv在y方向速度 单位:km/h

    double turnv;//agv转动角速率 单位:deg/s

    double compute_ActualSpd;
    ////////////////////////////////////////////////////////////////////////////////////////////////




    //AGV INFO 
    control_msgs::AGVStatus agvstatus;//agv前后轴转角信息
    AgvWheelSpd agvwheelspd;//agv轮速信息

    //subscribe and publish
    ros::Subscriber agvinfo_sub;
    ros::Subscriber agvwheelspd_sub;
    ros::Publisher  agv2p2info_pub;

};
 
}
}

#endif
