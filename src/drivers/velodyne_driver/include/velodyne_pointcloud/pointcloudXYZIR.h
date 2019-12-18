#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include <sensor_msgs/PointCloud2.h>
#include "velodyne_pointcloud/rawdata.h"

namespace velodyne_pointcloud
{
	class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase
	{
		public:
			velodyne_rawdata::VPointCloud pc;
			PointcloudXYZIR() {}

			virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity);
	};
}

#endif
