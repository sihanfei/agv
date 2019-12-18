/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle &node) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node);  // load lidar parameters
  node.param("model", model, std::string("RS32"));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  node.param("output_points_topic", output_points_topic, std::string("rslidar_points"));
  output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);

  // subscribe to rslidarScan packets
  std::string input_packets_topic;
  node.param("input_packets_topic", input_packets_topic, std::string("rslidar_packets"));
  rslidar_scan_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(uint32_t level)
{
  //ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model == "RS32")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }

  // process each packet provided by the driver

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints);
  }
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);

  output_.publish(outMsg);
}
}  // namespace rslidar_pointcloud
