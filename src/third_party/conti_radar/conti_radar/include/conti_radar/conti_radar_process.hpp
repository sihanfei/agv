#include <iostream>
#include <sys/time.h>
#include <signal.h>
#include <time.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include<chrono>

#include "conti_radar_can.hpp"           //can读取radar信息
#include "config.h"                      //配置文件读取

//Boost
#include <boost/thread.hpp>

//ROS
#include <ros/ros.h>
#include <conti_radar_msgs/ContiRadarObj.h>
#include <conti_radar_msgs/ContiRadarObjList.h>

using namespace std;
using namespace chrono;

class ContiRadarProcess
{

public:
     ContiRadarProcess(const ros::NodeHandle& nh_private,const ros::NodeHandle nh_node);
    ~ContiRadarProcess();
    int  conti_radar_start();
    void conti_radar_stop();
    int onRadarRecvCallback(const ContiRadar& radarmsg);
private:
    int init();
    void onContiRadarRecv();
   
    ContiRadarCan conti_radar_can_;        //从can读取radar的对象
    std::string can_channel_;              //can通道
    std::string config_file_;              //配置文件路径
    std::string frame_id_;

    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_node_;


    ros::Publisher pubContiRadarMsg_;

    conti_radar_msgs::ContiRadarObjList contiRadarObjList_;
};

//int CreatTimer(void (*timer_function)(union sigval v), unsigned int period);
//void* Thread_conti_radar_can_recv_Function(void* param);
