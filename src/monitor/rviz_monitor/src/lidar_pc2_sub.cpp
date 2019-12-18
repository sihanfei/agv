#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream> //使用stringstream需要引入这个头文件

#include <pcl/common/transforms.h> //Eigen

//////////////////spdlog//////////////////
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/logger.h"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#include "spdlog/async.h"
#include "spdlog/fmt/bin_to_hex.h"

#include <perception_sensor_msgs/LidarPointCloud.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class multiReceiver
{
public:
  multiReceiver()
  {
    sub  = nh.subscribe("/drivers/velodyne1/lidar_points", 1, &multiReceiver::Callback1, this);
    sub2 = nh.subscribe("/drivers/velodyne2/lidar_points", 1, &multiReceiver::Callback2, this);

    sub3 = nh.subscribe("/drivers/rs1/lidar_points", 1, &multiReceiver::Callback3, this);
    sub4 = nh.subscribe("/drivers/rs2/lidar_points", 1, &multiReceiver::Callback4, this);

    lidar1_pub1 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne1/agv_point_cloud", 1);
    lidar1_pub2 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne1/ground_point_cloud", 1);
    lidar1_pub3 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne1/object_point_cloud", 1);

    lidar2_pub1 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne2/agv_point_cloud", 1);
    lidar2_pub2 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne2/ground_point_cloud", 1);
    lidar2_pub3 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/velodyne2/object_point_cloud", 1);

    lidar3_pub1 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/rs1/agv_point_cloud", 1);
    lidar3_pub2 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/rs1/ground_point_cloud", 1);
    lidar3_pub3 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/rs1/object_point_cloud", 1);

    lidar4_pub1 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/rs2/agv_point_cloud", 1);
    lidar4_pub2 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/rs2/ground_point_cloud", 1);
    lidar4_pub3 = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/rs2/object_point_cloud", 1);
  }
  void Callback1(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg);
  void Callback2(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg);
  void Callback3(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg);
  void Callback4(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub2;

  ros::Subscriber sub3;
  ros::Subscriber sub4;

  ros::Publisher lidar1_pub1;
  ros::Publisher lidar1_pub2;
  ros::Publisher lidar1_pub3;

  ros::Publisher lidar2_pub1;
  ros::Publisher lidar2_pub2;
  ros::Publisher lidar2_pub3;

  ros::Publisher lidar3_pub1;
  ros::Publisher lidar3_pub2;
  ros::Publisher lidar3_pub3;

  ros::Publisher lidar4_pub1;
  ros::Publisher lidar4_pub2;
  ros::Publisher lidar4_pub3;
};

void multiReceiver::Callback1(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg)
{
  lidar1_pub1.publish(msg->agv_cloud_object);
  lidar1_pub2.publish(msg->point_cloud_ground);
  lidar1_pub3.publish(msg->point_cloud_object);
}

void multiReceiver::Callback2(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg)
{
  lidar2_pub1.publish(msg->agv_cloud_object);
  lidar2_pub2.publish(msg->point_cloud_ground);
  lidar2_pub3.publish(msg->point_cloud_object);
}

void multiReceiver::Callback3(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg)
{
  lidar3_pub1.publish(msg->agv_cloud_object);
  lidar3_pub2.publish(msg->point_cloud_ground);
  lidar3_pub3.publish(msg->point_cloud_object);
}

void multiReceiver::Callback4(const perception_sensor_msgs::LidarPointCloud::ConstPtr &msg)
{
  lidar4_pub1.publish(msg->agv_cloud_object);
  lidar4_pub2.publish(msg->point_cloud_ground);
  lidar4_pub3.publish(msg->point_cloud_object);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "lidar_pc2_pub");

  multiReceiver recOb;

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  // ros::spin();
  ros::waitForShutdown();
  //

  return 0;
}