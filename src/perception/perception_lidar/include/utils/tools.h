#ifndef _TOOLS_H
#define _TOOLS_H

#include <list>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <common_msgs/Point2D.h>
#include <common_msgs/DetectionInfo.h>
#include <perception_msgs/FusionDataInfo.h>
#include <perception_sensor_msgs/ObjectList.h>

#include "sensor_object/base_object.h"
#include "perception_lidar.h"

using namespace std;
using namespace Eigen;

namespace sensor_lidar
{
    class obstacleFeature;
}

// #define DEBUG_TOOLS
namespace sensor_lidar{
    void inputTypeTransform(list<sensor_lidar::obstacleFeature>& obstacleFeatureList, vector<sensor_lidar::BaseObject*> &sensor_obj_list, const ros::Time& sub_time);

    // transform "sensor_lidar::BaseObject" to "FusionDataInfo"
    void outputTypeTransform(const map<uint32_t, sensor_lidar::BaseObject*> &g_map, perception_sensor_msgs::ObjectList& pub_obj_list, ros::Time pub_time);

    // show fusion object in rviz
    void showResultInRviz(const map<uint32_t, sensor_lidar::BaseObject*> &g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point);

    void drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox);
}
#endif