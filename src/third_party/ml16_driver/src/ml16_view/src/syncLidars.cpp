#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h> 
#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h> 
#include <pcl/conversions.h>
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;

void callback(const PointCloud2ConstPtr& cloud_sub1,const PointCloud2ConstPtr& cloud_sub2)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(*cloud_sub1, *cloud_merge);
  pcl::fromROSMsg(*cloud_sub2, *cloud2);
  *cloud_merge += *cloud2;
  sensor_msgs::PointCloud2::Ptr cloud_sync = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_merge, *cloud_sync);
  pub.publish(cloud_sync);
}
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_node");
  ros::NodeHandle nh;
  pub=nh.advertise<sensor_msgs::PointCloud2> ("cloud_sync", 1);
  message_filters::Subscriber<PointCloud2> cloud_sub1(nh, "/ml16_cloud5", 10);
  message_filters::Subscriber<PointCloud2> cloud_sub2(nh, "/ml16_cloud6", 10);
  TimeSynchronizer<PointCloud2, PointCloud2> sync(cloud_sub1,cloud_sub2, 50);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::spin();
 
  return 0;
}
