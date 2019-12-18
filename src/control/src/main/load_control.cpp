#include "control.h"

#define NODE_NAME "load_control"

using namespace std;
using namespace control;

int main(int argc, char **argv)
{
  ROS_INFO("%s is star, ros node name is "NODE_NAME, argv[0]);

  //启动节点
  ros::init(argc, argv, "load_control");
  ros::NodeHandle n_tp;

  TPControl tp_control(n_tp);
    
  //主程序休眠
  ros::spin();

}
