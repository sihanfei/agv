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
#include <perception_camera/CameraObstacle.h>

#include "sensor_object/base_object.h"
#include "perception_camera.h"

using namespace cv;
using namespace std;
using namespace Eigen;

// #define DEBUG_TOOLS

namespace sensor_camera{
    void inputTypeTransform(const perception_camera::CameraObstacle& camera_obstacle_msg, vector<sensor_camera::BaseObject*> &sensor_obj_list, ros::Time sub_time);

    // transform "sensor_camera::BaseObject" to "FusionDataInfo"
    void outputTypeTransform(const map<uint32_t, sensor_camera::BaseObject*> &g_map, perception_sensor_msgs::ObjectList& pub_obj_list, ros::Time pub_time);

    // calculate 3d bbox
    void getWorldBbox(float x, float y, float w, float h, uint8_t classes, float state_3d[6]);

    // show fusion object in rviz
    void showResultInRviz(const map<uint32_t, sensor_camera::BaseObject*> &g_map, pcl::PointCloud<pcl::PointXYZRGB>& show_point, sensor_msgs::PointCloud2& msg_point);

    void drawLine(float x1, float y1, float z1, float x2, float y2, float z2, pcl::PointCloud<pcl::PointXYZRGB>&  pointCloudBoundingBox);
}
#endif