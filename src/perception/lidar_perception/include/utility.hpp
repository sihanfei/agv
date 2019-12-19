#ifndef LIDAR_OBSTACLE_UTILITY_HPP_
#define LIDAR_OBSTACLE_UTILITY_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <string>
#include <thread>

using namespace std;

typedef pcl::PointXYZI PointType;

string segmentedCloudTopic = "/segmented_cloud_pure";
float carHeight = 2.0;

typedef struct
{
  jsk_recognition_msgs::BoundingBox bounding_box;
  PointType min_pt;  // intensity = object_id
  PointType max_pt;
  PointType centroid_pt;  // intensity = uint object_speed;
} detectedObject, *detectedObjectPtr;

extern vector<detectedObject> global_objects;

#endif  // LIDAR_OBSTACLE_UTILITY_HPP_
