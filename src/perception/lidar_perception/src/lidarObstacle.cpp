#include "../include/utility.hpp"

class ObstacleDetection
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_segmented_cloud;
  ros::Publisher pub_objects_box;

  std_msgs::Header cloud_header;

  pcl::PointCloud<PointType>::Ptr segmented_cloud_ptr;  // 完整分割点云,intensity为点云id
  pcl::IndicesClustersPtr segmented_cloud_indices;      // 点云分割序号序列

  uint object_count;  // 本帧点云中的障碍物数量
  std::vector<detectedObject> current_objects;

public:
  ObstacleDetection() : nh("~")
  {
    segmentedCloudTopic = nh.param<string>("/basicParam/segmentedCloudTopic", "/segmented_cloud_pure");
    carHeight = nh.param<float>("/basicParam/carHeight", 1.5);

    sub_segmented_cloud =
        nh.subscribe<sensor_msgs::PointCloud2>(segmentedCloudTopic, 1, &ObstacleDetection::cloudHandler, this);

    pub_objects_box = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/objects_box_array", 1);

    allocateMemory();
    resetParameters();
  }

  ~ObstacleDetection()
  {
    ;
  }

  void allocateMemory()
  {
    segmented_cloud_ptr.reset(new pcl::PointCloud<PointType>());
    segmented_cloud_indices.reset(new pcl::IndicesClusters());
  }

  void resetParameters()
  {
    segmented_cloud_ptr->clear();
    segmented_cloud_indices->clear();

    current_objects.clear();

    object_count = 0;
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &segmentedCloudMsg)
  {
    copyPointCloud(segmentedCloudMsg);
    // 计算bbox
    getCurrentObjectsArray();
    // 计算特征
    publishResult();
    resetParameters();
  }

  void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr &segmentedCloudMsg)
  {
    cloud_header = segmentedCloudMsg->header;
    pcl::fromROSMsg(*segmentedCloudMsg, *segmented_cloud_ptr);

    // 首先将点云按照intensity划分开:因为lego中将intensity强制赋值为物体ID
    vector<uint> index_;
    size_t total_points_size_ = segmented_cloud_ptr->points.size();
    size_t cluster_points_count_ = 0;
    object_count = 0;

    // 先得到目标物的数量
    for (size_t i = 0; i < total_points_size_; ++i)
    {
      if (segmented_cloud_ptr->points[i].intensity > object_count)
      {
        object_count = segmented_cloud_ptr->points[i].intensity;
      }
    }
    // 根据目标物数量将点云index填入
    segmented_cloud_indices->resize(total_points_size_);
    for (size_t i = 0; i < total_points_size_; ++i)
    {
      int object_id_ = segmented_cloud_ptr->points[i].intensity;
      (*segmented_cloud_indices)[object_id_].header = segmented_cloud_ptr->header;
      (*segmented_cloud_indices)[object_id_].indices.push_back(i);
    }
  }

  void getCurrentObjectsArray()
  {
    pcl::ExtractIndices<PointType> extract_;
    pcl::PointCloud<PointType> single_object_cloud_;
    PointType object_minpt_, object_maxpt_, object_centroid_;
    detectedObject single_object_;

    extract_.setInputCloud(segmented_cloud_ptr);
    for (int i = 0; i < object_count; ++i)
    {
      pcl::PointIndicesPtr inlier_(new pcl::PointIndices((*segmented_cloud_indices)[i]));
      extract_.setIndices(inlier_);
      extract_.filter(single_object_cloud_);

      pcl::getMinMax3D(single_object_cloud_, object_minpt_, object_maxpt_);  // 最简单的AABB框
      pcl::computeCentroid(single_object_cloud_, object_centroid_);
      object_minpt_.intensity = i;     // id
      object_maxpt_.intensity = i;     // id
      object_centroid_.intensity = 0;  // speed
      single_object_.min_pt = object_minpt_;
      single_object_.max_pt = object_maxpt_;
      single_object_.centroid_pt = object_centroid_;

      single_object_.bounding_box.header = cloud_header;
      single_object_.bounding_box.label = i;  // current_id

      float length_ = object_maxpt_.x - object_minpt_.x;
      length_ = length_ > 0 ? length_ : length_ * -1;
      float width_ = object_maxpt_.y - object_minpt_.y;
      width_ = width_ > 0 ? width_ : width_ * -1;
      float height_ = object_maxpt_.z - object_minpt_.z;
      height_ = height_ > 0 ? height_ : height_ * -1;
      if (length_ < 0.01 || width_ < 0.01 || height_ < 0.01 || length_ > 100 || width_ > 100 || height_ > 100)
      {
        length_ = 1;
        width_ = 1;
        height_ = 1;
        ROS_INFO("\033[1;33m Over threshold \033[0m");
      }
      single_object_.bounding_box.dimensions.x = length_;
      single_object_.bounding_box.dimensions.y = width_;
      single_object_.bounding_box.dimensions.z = height_;
      // single_object_.bounding_box.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
      // single_object_.bounding_box.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
      // single_object_.bounding_box.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

      single_object_.bounding_box.pose.position.x = (object_maxpt_.x + object_minpt_.x) / 2;
      single_object_.bounding_box.pose.position.y = (object_maxpt_.y + object_minpt_.y) / 2;
      single_object_.bounding_box.pose.position.z = (object_maxpt_.z + object_minpt_.z) / 2;

      // single_object_.bounding_box.pose.orientation.w = 1.0;
      // single_object_.bounding_box.pose.orientation.x = 0.0;
      // single_object_.bounding_box.pose.orientation.y = 0.0;
      // single_object_.bounding_box.pose.orientation.z = 0.0;

      // single_object_.bounding_box.value = 0.0; // speed?

      current_objects.push_back(single_object_);
    }
  }

  void getObjectFeature(pcl::PointCloud<PointType> pointcloudin)
  {
  }

  void getObjectDescriptor()
  {
  }

  void publishResult()
  {
    if (pub_objects_box.getNumSubscribers() != 0)
    {
      jsk_recognition_msgs::BoundingBoxArray object_box_array_;
      for (int i = 0; i < object_count; ++i)
      {
        object_box_array_.boxes.push_back(current_objects[i].bounding_box);
      }
      object_box_array_.header = cloud_header;
      pub_objects_box.publish(object_box_array_);
    }
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
