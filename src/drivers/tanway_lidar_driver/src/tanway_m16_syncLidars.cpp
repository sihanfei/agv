
#include <endian.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;

// void callback(const PointCloud2ConstPtr &cloud_sub1, const PointCloud2ConstPtr &cloud_sub2,
//               const PointCloud2ConstPtr &cloud_sub3, const PointCloud2ConstPtr &cloud_sub4,
//               const PointCloud2ConstPtr &cloud_sub5, const PointCloud2ConstPtr &cloud_sub6)
// {
//   pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_merge = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
//   pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
//   pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud3      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
//   pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud4      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
//   pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud5      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
//   pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud6      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
//   pcl::fromROSMsg(*cloud_sub1, *cloud_merge);
//   pcl::fromROSMsg(*cloud_sub2, *cloud2);
//   pcl::fromROSMsg(*cloud_sub3, *cloud3);
//   pcl::fromROSMsg(*cloud_sub4, *cloud4);
//   pcl::fromROSMsg(*cloud_sub5, *cloud5);
//   pcl::fromROSMsg(*cloud_sub6, *cloud6);
//   *cloud_merge += *cloud2;
//   *cloud_merge += *cloud3;
//   *cloud_merge += *cloud4;
//   *cloud_merge += *cloud5;
//   *cloud_merge += *cloud6;
//   sensor_msgs::PointCloud2::Ptr cloud_sync = boost::make_shared< sensor_msgs::PointCloud2 >();
//   pcl::toROSMsg(*cloud_merge, *cloud_sync);
//   pub.publish(cloud_sync);
// }

void callback(const PointCloud2ConstPtr &cloud_sub1, const PointCloud2ConstPtr &cloud_sub2,
              const PointCloud2ConstPtr &cloud_sub3, const PointCloud2ConstPtr &cloud_sub4)
{
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud_merge = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud2      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud3      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();
  pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud4      = boost::make_shared< pcl::PointCloud< pcl::PointXYZRGB > >();

  pcl::fromROSMsg(*cloud_sub1, *cloud_merge);
  pcl::fromROSMsg(*cloud_sub2, *cloud2);
  pcl::fromROSMsg(*cloud_sub3, *cloud3);
  pcl::fromROSMsg(*cloud_sub4, *cloud4);

  *cloud_merge += *cloud2;
  *cloud_merge += *cloud3;
  *cloud_merge += *cloud4;

  sensor_msgs::PointCloud2::Ptr cloud_sync = boost::make_shared< sensor_msgs::PointCloud2 >();
  pcl::toROSMsg(*cloud_merge, *cloud_sync);
  pub.publish(cloud_sync);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tanway_m16_sync_node");
  ros::NodeHandle nh;
  pub = nh.advertise< sensor_msgs::PointCloud2 >("/drivers/tw_lidar/cloud_sync", 1);
  message_filters::Subscriber< PointCloud2 > cloud_sub1(nh, "/drivers/tw_lidar1/ml16_cloud", 100);
  message_filters::Subscriber< PointCloud2 > cloud_sub2(nh, "/drivers/tw_lidar2/ml16_cloud", 100);
  message_filters::Subscriber< PointCloud2 > cloud_sub3(nh, "/drivers/tw_lidar5/ml16_cloud", 100);
  message_filters::Subscriber< PointCloud2 > cloud_sub4(nh, "/drivers/tw_lidar6/ml16_cloud", 100);
  // message_filters::Subscriber< PointCloud2 > cloud_sub5(nh, "/drivers/tw_lidar/ml16_cloud5", 100);
  // message_filters::Subscriber< PointCloud2 > cloud_sub6(nh, "/drivers/tw_lidar/ml16_cloud6", 100);

  // typedef sync_policies::ApproximateTime< sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
  // sensor_msgs::PointCloud2,
  //                                         sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
  //                                         sensor_msgs::PointCloud2 >
  //     MySyncPolicy;
  // Synchronizer< MySyncPolicy > sync(MySyncPolicy(700), cloud_sub1, cloud_sub2, cloud_sub3, cloud_sub4, cloud_sub5,
  //                                   cloud_sub6);
  // sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

  typedef sync_policies::ApproximateTime< sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                          sensor_msgs::PointCloud2 >
      MySyncPolicy;
  Synchronizer< MySyncPolicy > sync(MySyncPolicy(700), cloud_sub1, cloud_sub2, cloud_sub3, cloud_sub4);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}