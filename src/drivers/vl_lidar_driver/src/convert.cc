#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

namespace velodyne_driver
{
Convert::Convert(ros::NodeHandle &node) : data_(new velodyne_rawdata::RawData())
{
  data_->setup(node);

  output_ = node.advertise< sensor_msgs::PointCloud2 >("velodyne_points", 10);

  velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Convert::processScan, ( Convert * )this,
                                  ros::TransportHints().tcpNoDelay(true));
}

void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
{
  if (output_.getNumSubscribers() == 0)
    return;

  PointcloudXYZIR out;
  out.pc.header.stamp    = pcl_conversions::toPCL(scanMsg->header).stamp;
  out.pc.header.frame_id = scanMsg->header.frame_id;
  out.pc.height          = 1;

  double timestamp = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    double packetTime = data_->unpack(scanMsg->packets[i], out);
    if (packetTime != 0)
    {
      timestamp = packetTime;
    }
  }

  sensor_msgs::PointCloud2 laserMsg;
  laserMsg.header.stamp    = ros::Time().fromSec(timestamp);
  laserMsg.header.frame_id = scanMsg->header.frame_id;
  pcl::toROSMsg(out.pc, laserMsg);
  output_.publish(laserMsg);
}
}
