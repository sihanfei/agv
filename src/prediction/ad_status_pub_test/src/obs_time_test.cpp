#include "common_msgs/DetectionInfo.h"
#include "control_msgs/ADControlAGV.h"
#include "geometry_msgs/PointStamped.h"
#include "location_msgs/FusionDataInfo.h"
#include "map_msgs/REFPointArray.h"
#include "perception_msgs/FusionDataInfo.h"
#include "perception_sensor_msgs/ObjectList.h"
#include "plan_msgs/DecisionInfo.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

ros::Subscriber velodyne_1_sub;
ros::Subscriber velodyne_2_sub;
ros::Subscriber fusion_velodyne_sub;
ros::Subscriber fusion_location_sub;
ros::Subscriber lidar_sub;
ros::Subscriber fusion_obs_sub;

ros::Time velodyne_1_time;
ros::Time velodyne_2_time;
ros::Time fusion_velodyne_time;
ros::Time fusion_location_time;
ros::Time lidar_time;
ros::Time fusion_obs_time;

int velodyne_1_recv_tip;
int velodyne_2_recv_tip;
int fusion_velodyne_tip;
int fusion_location_tip;
int lidar_tip;
int fusion_obs_tip;

void recvVelodyne1Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  velodyne_1_time     = msg->header.stamp;
  velodyne_1_recv_tip = 1;
}
void recvVelodyne2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  velodyne_2_time     = msg->header.stamp;
  velodyne_2_recv_tip = 1;
}
void recvFusionVelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  fusion_velodyne_time = msg->header.stamp;
  fusion_velodyne_tip  = 1;
}

void recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &msg)
{
  fusion_location_time = msg->header.stamp;
  fusion_location_tip  = 1;
}

void recvLidarObsCallback(const perception_sensor_msgs::ObjectList::ConstPtr &msg)
{
  lidar_time = msg->header.stamp;
  lidar_tip  = 1;
}

void recvObsCallback(const perception_msgs::FusionDataInfo::ConstPtr &msg)
{
  fusion_obs_time = msg->header.stamp;
  fusion_obs_tip  = 1;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "obs_test");
  ros::NodeHandle n;

  int mode_tip_1 = 0;
  if (argc == 2)
  {
    mode_tip_1 = atoi(argv[1]);
  }
  else
  {
    mode_tip_1 = 5;
  }

  velodyne_1_sub      = n.subscribe("/1/velodyne_points_201", 1, recvVelodyne1Callback);
  velodyne_2_sub      = n.subscribe("/2/velodyne_points_202", 1, recvVelodyne2Callback);
  fusion_velodyne_sub = n.subscribe("/drivers/velodyne/velodyne_points", 1, recvFusionVelodyneCallback);
  fusion_location_sub = n.subscribe("/localization/fusion_msg", 1, recvFusionLocationCallback);
  lidar_sub           = n.subscribe("/perception/detection_lidar", 1, recvLidarObsCallback);
  fusion_obs_sub      = n.subscribe("/perception/obstacle_info", 1, recvObsCallback);

  int count = 0;

  velodyne_1_recv_tip = 0;
  velodyne_2_recv_tip = 0;
  fusion_velodyne_tip = 0;
  fusion_location_tip = 0;
  lidar_tip           = 0;
  fusion_obs_tip      = 0;

  int be = 0;
  ros::Time cur_time;
  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    ++count;
    loop_rate.sleep();
    ros::spinOnce();
    int ans = velodyne_1_recv_tip + velodyne_2_recv_tip + fusion_velodyne_tip + fusion_location_tip + lidar_tip +
              fusion_obs_tip;
    if (ans == mode_tip_1)
    {
      cur_time = fusion_location_time;

      if (mode_tip_1 == 5)
      {
        ROS_INFO("Index %6d v1:%.4lf(s) v2:%.4lf(s) vf:%.4lf(s) fl:%.4lf(s) ld:%.4lf(s)", count,
                 velodyne_1_time.toSec() - cur_time.toSec(), velodyne_2_time.toSec() - cur_time.toSec(),
                 fusion_velodyne_time.toSec() - cur_time.toSec(), fusion_location_time.toSec() - cur_time.toSec(),
                 lidar_time.toSec() - cur_time.toSec());
      }
      if (mode_tip_1 == 6)
      {
        ROS_INFO("Index %6d v1:%.4lf(s) v2:%.4lf(s) vf:%.4lf(s) fl:%.4lf(s) ld:%.4lf(s) fo:%.4lf(s)", count,
                 velodyne_1_time.toSec() - cur_time.toSec(), velodyne_2_time.toSec() - cur_time.toSec(),
                 fusion_velodyne_time.toSec() - cur_time.toSec(), fusion_location_time.toSec() - cur_time.toSec(),
                 lidar_time.toSec() - cur_time.toSec(), fusion_obs_time.toSec() - cur_time.toSec());
      }

      velodyne_1_recv_tip = 0;
      velodyne_2_recv_tip = 0;
      fusion_velodyne_tip = 0;
      fusion_location_tip = 0;
      lidar_tip           = 0;
      fusion_obs_tip      = 0;
    }

    if (count > 20000)
    {
      break;
    }
  }

  ROS_INFO("shutting down!\n");
  ros::shutdown();

  return 0;
}
