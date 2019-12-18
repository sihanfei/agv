#include "planning.h"
#include "rtk_support.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_node");

  ros::NodeHandle n;

  ros::NodeHandle private_nh1("~ConfigParam1");
  ros::NodeHandle private_nh2("~ConfigParam2");

  //   if (argc <= 1)
  //   {
  //     ROS_ERROR("Planning_node:Please input parameters,1 means lattice plan,2 means support rtk.");
  //     ros::shutdown();
  //   }
  //   else
  //   {
  //     uint8_t mode = atoi(argv[1]);
  //     if (mode == 1)
  //     {
  //       ROS_INFO("Planning_node:Mode = %d,lattice plan.", mode);
  //       planning::Planning planning(n, private_nh1, private_nh2);
  //     }
  //     else if (mode == 2)
  //     {
  //       ROS_INFO("Planning_node:Mode = %d,support rtk.", mode);
  //       planning::RTKSupport rtk_support(n);
  //     }
  //   }

  planning::Planning planning(n, private_nh1, private_nh2);

  // ros::MultiThreadedSpinner spinner(4);
  ros::MultiThreadedSpinner spinner(10);
  spinner.spin();

  return 0;
}

// 2019-5-10
// add location jump judge
// 此为整个planning模块的入口，创建一个planning对象，整个程序的启动工作在Planning类的构造函数中完成
