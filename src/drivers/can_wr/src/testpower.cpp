#include "power_control_msgs/PowerControlCmd.h"
#include "ros/ros.h"

int main(int argc,char* argv[])
{
	for(int i = 0;i<argc;++i)
	{
		printf("argc:%d,argv:%d\n",argc,atoi(argv[i]));
	}
	ros::init(argc,argv,"test_sub");
        ros::NodeHandle nh;
	ros::Publisher node_state_pub = nh.advertise<power_control_msgs::PowerControlCmd>("/power_control/power_control_cmd", 1000, true);
	while(ros::ok())
    {
        power_control_msgs::PowerControlCmd PowerControlCmd_msg;
        PowerControlCmd_msg.length = 20;
	if(21 == argc)
	{
		for(int i = 0;i<20;++i)
		{
			PowerControlCmd_msg.data.push_back(atoi(argv[i+1]));
		}
	}
	else
	{
		for(int i = 0;i < 20; ++i)
		{
			PowerControlCmd_msg.data.push_back(0);
		}
	}
/*        for(int i = 0;i < 20; ++i)
        {
            if(15 == i)
            {
                PowerControlCmd_msg.data.push_back(1);
            }
            else
            {

                PowerControlCmd_msg.data.push_back(0);
            }
        }
*/
        node_state_pub.publish(PowerControlCmd_msg);
        sleep(1);
    }

	return 0;
}
