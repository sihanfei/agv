#ifndef DRIVERS_RELAY_H_
#define DRIVERS_RELAY_H_

#include <ros/ros.h>
#include "power_control_msgs/PowerControlCmd.h"

namespace superg_agv
{
namespace drivers
{

class Relay
{
    public:
        Relay();
        ~Relay();
        void openCanRelay(int channel,int state);
    private:
        ros::NodeHandle nh_1;
        ros::Publisher pub_canrelay;
};

}
}

#endif
