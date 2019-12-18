#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H


#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
	struct PointXYZIR
	{
		PCL_ADD_POINT4D;
		float intensity;
		uint16_t ring;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	} EIGEN_ALIGN16;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR, 
				  (float, x, x)
				  (float, y, y)
				  (float, z, z)
				  (float, intensity, intensity)
				  (uint16_t, ring, ring))

#endif
