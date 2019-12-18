#include "../include/utility.hpp"

class ObstacleDetection
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_segmented_cloud;

  std_msgs::Header cloud_header;

  pcl::PointCloud< PointType >::Ptr segmented_cloud_ptr; // 完整分割点云,intensity为点云id
  pcl::IndicesClustersPtr segmented_cloud_indices;       // 点云分割序号序列

  uint object_count; // 本帧点云中的障碍物数量
  std::vector< detectedObject > current_objects;

public:
  ObstacleDetection() : nh("~")
  {
    sub_segmented_cloud =
        nh.subscribe< sensor_msgs::PointCloud2 >(segmentedCloudTopic, 1, &ObstacleDetection::cloudHandler, this);

    allocateMemory();
    resetParameters();
  }

  ~ObstacleDetection()
  {
    ;
  }

  void allocateMemory()
  {
    segmented_cloud_ptr.reset(new pcl::PointCloud< PointType >());
    segmented_cloud_indices.reset(new pcl::IndicesClusters());
  }

  void resetParameters()
  {
    segmented_cloud_ptr->clear();
    segmented_cloud_indices->clear();
    object_count = 1;
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &segmentedCloudMsg)
  {
    copyPointCloud(segmentedCloudMsg);
    resetParameters();
  }

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &segmentedCloudMsg)
  {
    cloud_header = segmentedCloudMsg->header;
    pcl::fromROSMsg(*segmentedCloudMsg, *segmented_cloud_ptr);
  }

  void getObjectBox(pcl::PointCloud< PointType > pointcloudin)
  {
  }

  void getObjectFeature(pcl::PointCloud< PointType > pointcloudin)
  {
  }

  void getObjectDescriptor()
  {
    // 首先将点云按照intensity划分开:因为lego中将intensity强制赋值为物体ID
    vector< uint > index_;
    size_t total_points_size_    = segmented_cloud_ptr->points.size();
    size_t cluster_points_count_ = 0;
    object_count                 = 0;
    while (cluster_points_count_ != total_points_size_)
    {
      index_.clear();
      ++object_count;
      for (size_t i = 0; i < total_points_size_; ++i)
      {
        PointType this_point_ = segmented_cloud_ptr->points[i];
        if (this_point_.intensity == object_count)
        {
          index_.push_back(i);
          ++cluster_points_count_;
        }
      }
      pcl::PointIndices object_indices_;
      object_indices_.indices.resize(index_.size());
      for (size_t i = 0; i < index_.size(); ++i)
      {
        object_indices_.indices[i] = index_[i];
      }
      object_indices_.header = segmented_cloud_ptr->header;
      segmented_cloud_indices->push_back(object_indices_);
    }
    ROS_INFO("\033[1;32m --> \033[0m get %i objects", object_count);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_perception");

  ObstacleDetection OD;
  ROS_INFO("\033[1;32m---->\033[0m Obstacle Detection Started.");

  ros::spin();

  return 0;
}
