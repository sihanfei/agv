#include "utils.h"


int main(int argc, char **argv)
{
	
  ros::init(argc, argv, "decision"); 	// 新建ros node节点名：decision

  ros::NodeHandle n;

  pnc::Decision decision(n);

  ros::MultiThreadedSpinner spinner(10);

  spinner.spin();

  return 0;

}
