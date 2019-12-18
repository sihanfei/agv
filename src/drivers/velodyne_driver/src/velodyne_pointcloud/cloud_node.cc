#include <ros/ros.h>
#include <iostream>
#include "velodyne_pointcloud/convert.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_node");
	ros::NodeHandle node;

	velodyne_pointcloud::Convert conv(node);
	
	ros::spin();
	return 0;
}
