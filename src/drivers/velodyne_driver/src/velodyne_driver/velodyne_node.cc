#include <ros/ros.h>
#include <iostream>
#include "velodyne_driver/driver.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "velodyne_node");
    ros::NodeHandle nh;

	velodyne_driver::VelodyneDriver dvr(nh);

	while(ros::ok() && dvr.poll())
	{
		ros::spinOnce();
	}
	return 0;
}
