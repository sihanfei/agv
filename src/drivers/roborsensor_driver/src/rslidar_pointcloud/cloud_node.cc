/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw RSLIDAR LIDAR packets to PointCloud2.

*/
#include "convert.h"
//#include <ros/ros.h>

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_node");
  ros::NodeHandle node;

  // create conversion class, which subscribes to raw data
  rslidar_pointcloud::Convert conv(node);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
