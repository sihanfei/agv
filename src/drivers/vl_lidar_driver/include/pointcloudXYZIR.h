#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include "rawdata.h"
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_driver
{
class PointcloudXYZIR : public velodyne_driver::DataContainerBase
{
public:
  velodyne_driver::VPointCloud pc;
  PointcloudXYZIR()
  {
  }

  virtual void addPoint(const float &x, const float &y, const float &z, const uint16_t &ring, const uint16_t &azimuth,
                        const float &distance, const float &intensity);
};
}

#endif
