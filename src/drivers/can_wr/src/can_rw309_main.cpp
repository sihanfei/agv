#include "can_wr309.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "can_rw309");
    ros::NodeHandle nh;
    adcuSDKInit();

    CANDrivers309 candrivers309;
    usleep(1000 * 2000);

    int channel;
    int devid = 1;
}