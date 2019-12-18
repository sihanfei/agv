#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#define PI 3.14159265

using namespace std;

extern const string imuTopic = "/imu/data";

// Save pcd
extern const string fileDirectory = "/home/ads/data/TW/";

// TW-16
typedef pcl::PointXYZI PointType;
extern const string pointCloudTopic = "/tensorpro_cloud";

extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 566;
extern const float ang_res_x = 120 / float(Horizon_SCAN);
extern const float ang_res_y = 11 / float(N_SCAN - 1);
extern const float ang_bottom = 5.5 + 0.1;
extern const int groundScanInd = N_SCAN / 2 - 1;

extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMountAngle = 0.0;
float segmentTheta = 40.0 / 180.0 * M_PI;  // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;

extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0;  // key frame that is within n meters from current pose will
                                                            // be considerd for scan-to-map optimization (when loop
                                                            // closure disabled)
extern const int surroundingKeyframeSearchNum = 50;         // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius =
    7.0;  // key frame that is within n meters from current pose will be considerd for loop closure
extern const int historyKeyframeSearchNum =
    25;  // 2n+1 number of history key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3;  // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 500.0;  // key frames with in n meters will be visualized

struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const &left, smoothness_t const &right)
  {
    return left.value < right.value;
  }
};

struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

#endif
