
#ifndef __VELODYNE_POINTCLOUD_CONVERT_H
#define __VELODYNE_POINTCLOUD_CONVERT_H

#include "pointcloudXYZIR.h"
#include "rawdata.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_driver
{
class Convert
{
public:
  Convert(ros::NodeHandle &node);
  ~Convert()
  {
  }

private:
  void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

  boost::shared_ptr< velodyne_rawdata::RawData > data_;

  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;
};
}

#endif
