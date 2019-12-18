#include "relay.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "relay_control");
	ros::NodeHandle n;
    if(argc > 0)
    {
        ROS_INFO_STREAM("argc:" << argc << ",1:" << atoi(argv[1]) << ",2:" << atoi(argv[2]));
        int channel = atoi(argv[1]);
        int state   = atoi(argv[2]);
        if(channel >= 20 || channel < 0)
        {
            ROS_INFO("please enter channel(0 ~ 19)!");
            return -1;
        }
        if(state > 1 || state < 0)
        {
            ROS_INFO("please enter state(0 or 1)!");
            return -1;
        }
        Relay relay;
        relay.openCanRelay(channel,state);
        ROS_INFO("send success!!!");
    }
    else
    {
        ROS_INFO("please enter channel and state!!!");
    }
    return 0;
}