#include "velodyne_pointcloud/pointcloudXYZIR.h"

namespace velodyne_pointcloud
{
	void PointcloudXYZIR::addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& /*azimuth*/, const float& /*distance*/, const float& intensity)
	{
		velodyne_rawdata::VPoint point;
		point.ring = ring;
		point.x = x;
		point.y = y;
		point.z = z;
		point.intensity = intensity;

		pc.points.push_back(point);
		++pc.width;
	}
}
