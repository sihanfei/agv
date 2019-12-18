#include "relay.h"

namespace superg_agv
{
namespace drivers
{

Relay::Relay()
{
    pub_canrelay = nh_1.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
}

Relay::~Relay()
{

}

void Relay::openCanRelay(int channel,int state)
{
    // ros::NodeHandle nh_1;
    // ros::Publisher pub_canrelay = nh_1.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
    power_control_msgs::PowerControlCmd PowerControlCmd_msg;
    PowerControlCmd_msg.header.stamp            = ros::Time::now();
    PowerControlCmd_msg.header.frame_id         = "base_link";
    PowerControlCmd_msg.length                  = 20;
    for(int i = 0;i < 20;++i)
    {
        if(channel == i)
        {
        PowerControlCmd_msg.data.push_back(1);
        // ROS_INFO("1");
        }
        else
        {
        PowerControlCmd_msg.data.push_back(2);
        // ROS_INFO("2");
        }
    }
    pub_canrelay.publish(PowerControlCmd_msg);
    // ROS_INFO("pub_canrelay");
}

}
}