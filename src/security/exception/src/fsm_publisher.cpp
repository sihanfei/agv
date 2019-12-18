#include "ros/ros.h"
#include "ros/console.h"
//#include "exception.h"
#include "exception/VMSControlAD.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fsm_publisher");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<exception::VMSControlAD>("/fsm/to_exception_order",10);
  ros::Rate rate(100);
  

  while(ros::ok())
  {
    exception::VMSControlAD fsm_info;
    fsm_info.header.stamp = ros::Time::now(); 
    fsm_info.fsm_control = 1;
    fsm_info.CMD_TYpe = 0;
    fsm_info.receiver = 3;
    fsm_info.message_num = 1;
    fsm_info.message_code.emplace_back("E0801002");

    pub.publish(fsm_info);
    rate.sleep();
  }

  return 0;
}