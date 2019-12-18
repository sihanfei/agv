#include <ros/ros.h>

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <vector>

using namespace std;

#define NODE_NAME "web_service_node"

int g_temp;

int main(int argc, char **argv)
{
  ROS_INFO("ROS node is star, name is [%s], file name is %s", NODE_NAME, argv[0]);

  std::stringstream ss;
  ss << NODE_NAME << "_" << getpid();
  ros::init(argc, argv, ss.str());
  // ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  ros::Subscriber sub_1_ = nh.subscribe("/localization/fusion_msg", 1, &Callback1);
  ros::Subscriber sub_2_ = nh.subscribe("/perception/rviz_obstacle_cam_lidar_msg", 1, &lidar2Callback);
  ros::Subscriber sub_3_ = nh.subscribe("/rs1/rslidar_points", 1, &lidar3Callback);
  ros::Subscriber sub_4_ = nh.subscribe("/rs2/rslidar_points", 1, &lidar4Callback);
  pub                    = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne/velodyne_points", 100);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  ros::waitForShutdown();
  return 0;
}
