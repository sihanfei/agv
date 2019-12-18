#ifndef _PERCEPTION_FUSION_H_
#define _PERCEPTION_FUSION_H_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <map>
#include <vector>
#include "omp.h"

#include <common_msgs/DetectionInfo.h>
#include <common_msgs/ObstacleInfo.h>
#include <location_msgs/FusionDataInfo.h>
#include <perception_msgs/FusionDataInfo.h>
#include <perception_sensor_msgs/LidarPointCloud.h>
#include <perception_sensor_msgs/ObjectList.h>

#include "associate/base_association.h"
#include "sensor_object/base_object.h"
#include "utils/tools.h"

using namespace std;

#define PI 3.14159265

// #define DEBUG_PERCEPTION_FUSION

namespace sensor_lidar
{
class obstacleFeature
{
public:
  explicit obstacleFeature()
  {
  }
  explicit obstacleFeature(float xMax, float xMin, float yMax, float yMin, float x1, float y1, float x2, float y2,
                           float x3, float y3, float x4, float y4)
    : xMax_(xMax)
    , xMin_(xMin)
    , yMax_(yMax)
    , yMin_(yMin)
    , x1_(x1)
    , y1_(y1)
    , x2_(x2)
    , y2_(y2)
    , x3_(x3)
    , y3_(y3)
    , x4_(x4)
    , y4_(y4)
  {
  }

  ~obstacleFeature()
  {
  }

public:
  float xMax_ = 0;
  float xMin_ = 0;
  float yMax_ = 0;
  float yMin_ = 0;
  float x1_ = 0;
  float y1_ = 0;
  float x2_ = 0;
  float y2_ = 0;
  float x3_ = 0;
  float y3_ = 0;
  float x4_ = 0;
  float y4_ = 0;
};

class PerceptionLidar
{
public:
  explicit PerceptionLidar(BaseAssociation *base_associatio, float cluster_Tolerance, int min_cluster_size,
                           int max_cluster_size, bool is_draw);
  ~PerceptionLidar();

protected:
  void callbackLidarNoSync0(const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr);
  void callbackLidarNoSync1(const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr);
  void callbackLidarNoSync2(const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr);
  void callbackLidarNoSync3(const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr);
  void PerceptionLidarFunction(const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr);
  void updateNewObject(vector<sensor_lidar::BaseObject *> &obj_list);
  void updateAssociatedObject(vector<sensor_lidar::BaseObject *> &new_obj, Eigen::MatrixXd &matrix);
  void updateUnassociatedObject(vector<sensor_lidar::BaseObject *> &new_obj, Eigen::MatrixXd &matrix);
  void publishFusionObject(ros::Time pub_time, bool is_draw);
  void read_msgs(const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr);
  void cutGround1();
  void ibeoFilter();
  void euclideanCluster();
  bool overlapAGV(const obstacleFeature &oneObstacleFeature);
  bool overlapAGV(const float x2_max, const float x2_min, const float y2_max, const float y2_min);
  void boundingBoxSegmentation(const pcl::PointIndices &cluster_indice, const float alpha, const float yMax,
                               const float yMin, obstacleFeature &oneObstacleFeature1,
                               obstacleFeature &oneObstacleFeature2);

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_obstacle_info_;
  ros::Publisher pub_rviz_bounding_box_;
  ros::Publisher pub_rviz_bounding_box_info_;

  // ros::Subscriber sub_pointcloud_;
  // ros::Subscriber sub_location_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, location_msgs::FusionDataInfo>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud_;
  message_filters::Subscriber<location_msgs::FusionDataInfo> sub_location_;
  boost::shared_ptr<Sync> sync_pointcloud_location_;

  ros::Subscriber sub_pointcloud_no_sync_0;
  ros::Subscriber sub_pointcloud_no_sync_1;
  ros::Subscriber sub_pointcloud_no_sync_2;
  ros::Subscriber sub_pointcloud_no_sync_3;

  BaseAssociation *base_association_;
  map<uint32_t, sensor_lidar::BaseObject *> global_object_;

  uint32_t global_id_ = 0;
  bool is_draw_ = false;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr ibeoPointCloud{new pcl::PointCloud< pcl::PointXYZ >};
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudcutground{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudfilted0{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudfilted1{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudfilted2{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudfilted3{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudfilted{ new pcl::PointCloud<pcl::PointXYZ> };
  std::list<obstacleFeature> obstacleFeatureListPointer;

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzfilteredCloudPtr{ new pcl::PointCloud<pcl::PointXYZ> };

  float cluster_Tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  // perception_sensor_msgs::ObjectList global_pub_object_list;
  float att[3];
  Eigen::Vector3d veh_llh;
  float velocity_xyz[3];
  uint8_t lidar_number;
};
}

#endif  // _PERCEPTION_FUSION_H_
