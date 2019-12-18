#include "exception.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "exception_node");

  ros::NodeHandle node;

  exception::Exception exception(node,argv[0]);

  ros::spin();

  return 0;
}


