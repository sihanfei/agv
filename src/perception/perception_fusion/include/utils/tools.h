#ifndef _TOOLS_H
#define _TOOLS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common_msgs/Point2D.h>
#include <common_msgs/DetectionInfo.h>
#include <perception_msgs/FusionDataInfo.h>
#include <perception_sensor_msgs/ObjectList.h>

#include "sensor_object/base_object.h"

using namespace std;
using namespace Eigen;

// #define DEBUG_TOOLS

// transform "ObjectList" to "BaseObject"
void inputTypeTransform(const perception_sensor_msgs::ObjectList &obj_list, vector<BaseObject*>& sensor_obj_list, uint8_t sensor_type, float (&att)[3], Vector3d &veh_llh);

// transform "BaseObject" to "FusionDataInfo"
// void outputTypeTransform(const map<uint32_t, BaseObject*> &g_map, perception_msgs::FusionDataInfo& pub_obj_list, ros::Time pub_time, float veh_vxvy[2], float att[3]);

// show fusion object in rviz
// void showResultInRviz(const map<uint32_t, BaseObject*> &g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, float att[3]);

// void showResultInRviz2(perception_msgs::FusionDataInfo &pub_obj_list, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point);

void drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox);

void drawLineCamera(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox);

void outputTypeTransformCamera(const map<uint32_t, BaseObject*> &publish_map, perception_msgs::FusionDataInfo &pub_obj_list, const ros::Time pub_time, const float veh_vxvy[2], const float att[3]);

// void outputTypeTransformCamera(const map<uint32_t, BaseObject*> &g_map, perception_msgs::FusionDataInfo &pub_obj_list, ros::Time pub_time, float veh_vxvy[2], float att[3], map<uint32_t, BaseObject*> eryuan_object_list[], int eryuan_number);

// void outputTypeTransformCamera(const map<uint32_t, BaseObject*> &g_map, perception_msgs::FusionDataInfo &pub_obj_list, ros::Time pub_time, float veh_vxvy[2], float att[3], vector<BaseObject*> base_object_list);

void showResultInRvizWithCamera(const map<uint32_t, BaseObject*> &publish_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, const float att[3], const bool global_rviz);

// void showResultInRvizWithCamera(const map<uint32_t, BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, float att[3], map<uint32_t, BaseObject*> eryuan_object_list[], int eryuan_number, bool global_rviz);

// void showResultInRvizWithCamera(const map<uint32_t, BaseObject*>& g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point, float att[3], vector<BaseObject*> base_object_list);

// void filterEryuanObject(map<uint32_t, BaseObject*> eryuan_object_list[], int sensor_index);

void updateEryuanObject(map<uint32_t, BaseObject*> eryuan_object_list[], const perception_sensor_msgs::ObjectList msg_object_list, float att[], Vector3d &veh_llh,
float range_xmax = 5.0, float range_xmin = -5.0, float range_ymax = 15.0, float range_ymin = -15.0,
float agv_xmax = 1.8, float agv_xmin = -1.8, float agv_ymax = 8.0, float agv_ymin = -8.0);

void removeObject(const ros::Time lidar_time, const ros::Time eryuan_time, map<uint32_t, BaseObject*> &g_map, map<uint32_t, BaseObject*> eryuan_object_list[], const int eryuan_number, map<uint32_t, BaseObject*> &publish_map);

#endif