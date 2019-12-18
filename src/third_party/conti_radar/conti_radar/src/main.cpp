#include <ros/ros.h>
#include <iostream>
#include "conti_radar_process.hpp"
using namespace std;

int main(int argc, char** argv)
{
    ROS_INFO("*******        Start radar!          **********");

    ros::init(argc, argv, "conti_radar");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_node;

    ContiRadarProcess conti_radar_process(nh_private,nh_node);

    conti_radar_process.conti_radar_start();

    ros::AsyncSpinner spinner(2);
	spinner.start();
  	ros::waitForShutdown();

    return 0;
}