#include "ros/ros.h"
#include "ros/console.h"

#include "exception/AGVStatus.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agv_status_publisher");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<exception::AGVStatus>("/drivers/com2agv/agv_status",10);
  ros::Rate rate(100);
  exception::AGVStatus agv_status_info;

  while(ros::ok())
  {
    agv_status_info.header.stamp = ros::Time::now(); 
    agv_status_info.ActualSpd = 0;
    agv_status_info.EStopStatus = 1;

    pub.publish(agv_status_info);
    rate.sleep();
  }

  return 0;
}