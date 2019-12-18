#include "perception_lidar.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/flann_search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

sensor_lidar::PerceptionLidar::PerceptionLidar(BaseAssociation *base_association, float cluster_Tolerance,
                                               int min_cluster_size, int max_cluster_size, bool is_draw)
    : base_association_(base_association), cluster_Tolerance_(cluster_Tolerance), min_cluster_size_(min_cluster_size),
      max_cluster_size_(max_cluster_size), is_draw_(is_draw)
{
#ifdef DEBUG_PERCEPTION_FUSION
  cout << "PerceptionLidar ctor start" << endl;
#endif

  pub_obstacle_info_     = nh_.advertise< perception_sensor_msgs::ObjectList >("/perception/detection_lidar", 100);
  pub_rviz_bounding_box_ = nh_.advertise< sensor_msgs::PointCloud2 >("/perception_lidar/rviz/make_bounding_box", 2);
  pub_rviz_bounding_box_info_ =
      nh_.advertise< visualization_msgs::Marker >("/perception_lidar/rviz/make_bounding_box_info", 2);

  // sub_location_ = nh_.subscribe("/localization/fusion_msg", 1, &sensor_lidar::PerceptionLidar::callbackLocation,
  // this); sub_pointcloud_ = nh_.subscribe("/drivers/velodyne/velodyne_points", 1,
  // &sensor_lidar::PerceptionLidar::callbackLidar, this); sub_pointcloud_ = nh_.subscribe("/velodyne_points", 1,
  // &sensor_lidar::PerceptionLidar::callbackLidar, this);

  // sub_pointcloud_.subscribe(nh_, "/drivers/velodyne/velodyne_points", 1);
  // sub_location_.subscribe(nh_, "/localization/fusion_msg", 1);
  // sync_pointcloud_location_.reset(new Sync(MySyncPolicy(5000), sub_pointcloud_, sub_location_));
  // sync_pointcloud_location_->registerCallback(boost::bind(&sensor_lidar::PerceptionLidar::callbackLidar, this, _1,
  // _2));

  sub_pointcloud_no_sync_0 =
      nh_.subscribe("/drivers/velodyne1/lidar_points", 1, &PerceptionLidar::callbackLidarNoSync0, this);
  sub_pointcloud_no_sync_1 =
      nh_.subscribe("/drivers/velodyne2/lidar_points", 1, &PerceptionLidar::callbackLidarNoSync1, this);
  sub_pointcloud_no_sync_2 =
      nh_.subscribe("/drivers/rs1/lidar_points", 1, &PerceptionLidar::callbackLidarNoSync2, this);
  sub_pointcloud_no_sync_3 =
      nh_.subscribe("/drivers/rs2/lidar_points", 1, &PerceptionLidar::callbackLidarNoSync3, this);

#ifdef DEBUG_PERCEPTION_FUSION
  cout << "PerceptionLidar ctor endl" << endl;
#endif
}

sensor_lidar::PerceptionLidar::~PerceptionLidar()
{
  delete base_association_;
}

/*void sensor_lidar::PerceptionLidar::callbackLocation(const location_msgs::FusionDataInfo::ConstPtr& location_msg)
{
    att[0] = location_msg->yaw;
    att[1] = location_msg->pitch;
    att[2] = location_msg->roll;
    veh_llh[0] = location_msg->pose.x;
    veh_llh[1] = location_msg->pose.y;
    veh_llh[2] = location_msg->pose.z;
}*/

// 传感器类型 0：相机, 1: 激光雷达, 2: 毫米板雷达
void sensor_lidar::PerceptionLidar::callbackLidarNoSync0(
    const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr)
{
  lidar_number = 0;
  PerceptionLidarFunction(lidar_pointcloud_msgs_ptr);
}

// 传感器类型 0：相机, 1: 激光雷达, 2: 毫米板雷达
void sensor_lidar::PerceptionLidar::callbackLidarNoSync1(
    const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr)
{
  lidar_number = 1;
  PerceptionLidarFunction(lidar_pointcloud_msgs_ptr);
}

// 传感器类型 0：相机, 1: 激光雷达, 2: 毫米板雷达
void sensor_lidar::PerceptionLidar::callbackLidarNoSync2(
    const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr)
{
  lidar_number = 2;
  PerceptionLidarFunction(lidar_pointcloud_msgs_ptr);
}

// 传感器类型 0：相机, 1: 激光雷达, 2: 毫米板雷达
void sensor_lidar::PerceptionLidar::callbackLidarNoSync3(
    const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr)
{
  lidar_number = 3;
  PerceptionLidarFunction(lidar_pointcloud_msgs_ptr);
}

void sensor_lidar::PerceptionLidar::PerceptionLidarFunction(
    const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr)
{
  cout << "====================== callbackLidarNoSync: start ======================" << endl;
  std::cout << "pointcloud time = " << std::setprecision(15) << lidar_pointcloud_msgs_ptr->header.stamp << std::endl;
  std::cout << "start time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;

  clock_t start, end;
  float elapsedTime;
  start = clock();

  read_msgs(lidar_pointcloud_msgs_ptr);

  cutGround1();
  ibeoFilter();
  std::cout << "The size of point cloud (filted) is: " << PointCloudfilted->points.size() << std::endl;
  // if(obstacleFeatureListPointer != nullptr)
  //     delete obstacleFeatureListPointer;
  obstacleFeatureListPointer.clear();
  euclideanCluster();
  end         = clock();
  elapsedTime = ( float )(end - start) / CLOCKS_PER_SEC;
  cout << "euclideanCluster in: " << elapsedTime << " s" << endl;

  // 关联矩阵
  Eigen::MatrixXd incidence_matrix;

  // 将目标信息转为BaseObject类型
  vector< BaseObject * > base_object_list;
  inputTypeTransform(obstacleFeatureListPointer, base_object_list, lidar_pointcloud_msgs_ptr->header.stamp);

  // 处理第一帧数据
  if (global_object_.empty())
  {
    updateNewObject(base_object_list);
  }
  else
  {
    cout << "base_object_list.size: " << base_object_list.size() << endl;

    if (0 != base_object_list.size())
    {
      clock_t start2, end2;
      float elapsedTime2;
      start2 = clock();
      // 对疑似新目标和未匹配全局目标做数据关联
      base_association_->getIncidenceMatrix(global_object_, base_object_list, incidence_matrix);
      end2         = clock();
      elapsedTime2 = ( float )(end2 - start2) / CLOCKS_PER_SEC;
      cout << "Completed2 in: " << elapsedTime2 << " s" << endl;

      // 更新关联上的全局目标
      start2 = clock();
      updateAssociatedObject(base_object_list, incidence_matrix);
      end2         = clock();
      elapsedTime2 = ( float )(end2 - start2) / CLOCKS_PER_SEC;
      cout << "Completed3 in: " << elapsedTime2 << " s" << endl;

      // 更新新目标
      start2 = clock();
      updateUnassociatedObject(base_object_list, incidence_matrix);
      end2         = clock();
      elapsedTime2 = ( float )(end2 - start2) / CLOCKS_PER_SEC;
      cout << "Completed3 in: " << elapsedTime2 << " s" << endl;
    }
  }

  publishFusionObject(lidar_pointcloud_msgs_ptr->header.stamp, is_draw_);

  base_object_list.clear();

  end         = clock();
  elapsedTime = ( float )(end - start) / CLOCKS_PER_SEC;
  cout << "Completed in: " << elapsedTime << " s" << endl;

  std::cout << "end time = " << std::setprecision(15) << ros::Time::now().toSec() << std::endl;
  cout << "====================== callbackLidarNoSync: end ======================" << endl;
}

void sensor_lidar::PerceptionLidar::read_msgs(
    const perception_sensor_msgs::LidarPointCloud::ConstPtr lidar_pointcloud_msgs_ptr)
{
  // global_pub_object_list.header = lidar_pointcloud_msgs_ptr->header;
  // global_pub_object_list.yaw = lidar_pointcloud_msgs_ptr->location_start.yaw;
  // global_pub_object_list.pitch = lidar_pointcloud_msgs_ptr->location_start.pitch;
  // global_pub_object_list.roll = lidar_pointcloud_msgs_ptr->location_start.roll;
  // global_pub_object_list.pose.x = lidar_pointcloud_msgs_ptr->location_start.pose.x;
  // global_pub_object_list.pose.y = lidar_pointcloud_msgs_ptr->location_start.pose.y;
  // global_pub_object_list.pose.z = lidar_pointcloud_msgs_ptr->location_start.pose.z;
  // global_pub_object_list.velocity.linear.x = lidar_pointcloud_msgs_ptr->location_start.velocity.linear.x;
  // global_pub_object_list.velocity.linear.y = lidar_pointcloud_msgs_ptr->location_start.velocity.linear.y;
  // global_pub_object_list.velocity.linear.z = lidar_pointcloud_msgs_ptr->location_start.velocity.linear.z;
  att[0]          = lidar_pointcloud_msgs_ptr->location_start.yaw;
  att[1]          = lidar_pointcloud_msgs_ptr->location_start.pitch;
  att[2]          = lidar_pointcloud_msgs_ptr->location_start.roll;
  veh_llh[0]      = lidar_pointcloud_msgs_ptr->location_start.pose.x;
  veh_llh[1]      = lidar_pointcloud_msgs_ptr->location_start.pose.y;
  veh_llh[2]      = lidar_pointcloud_msgs_ptr->location_start.pose.z;
  velocity_xyz[0] = lidar_pointcloud_msgs_ptr->location_start.velocity.linear.x;
  velocity_xyz[1] = lidar_pointcloud_msgs_ptr->location_start.velocity.linear.y;
  velocity_xyz[2] = lidar_pointcloud_msgs_ptr->location_start.velocity.linear.z;
  ROS_INFO("I recevied point cloud!");
  pcl::fromROSMsg(lidar_pointcloud_msgs_ptr->point_cloud_object, *original_pointcloud);
  std::cout << "The size of point cloud is: " << original_pointcloud->points.size() << std::endl;
}

void sensor_lidar::PerceptionLidar::cutGround1()
{
  xyzCloudPtr->clear();
  PointCloudcutground->clear();
  for (pcl::PointXYZ point : *original_pointcloud)
  {
    if (point.x < 10.0 && point.x > -10.0 && point.y < 25.0 && point.y > -25.0)
    {
      if ((point.z <= 100 && point.z >= -2.0) && (point.x <= 25 && point.x >= -25) && (point.y <= 50 && point.y >= -50))
      {
        xyzCloudPtr->push_back(point);
        point.z = 0;
        PointCloudcutground->points.push_back(point);
      }
    }
    else
    {
      if ((point.z <= 100 && point.z >= -2.0) && (point.x <= 25 && point.x >= -25) && (point.y <= 50 && point.y >= -50))
      {
        xyzCloudPtr->push_back(point);
        point.z = 0;
        PointCloudcutground->points.push_back(point);
      }
    }
  }
  // for (int j = 0; j < PointCloudcutground->points.size(); j++)
  // {
  //     PointCloudcutground->points[j].z = 0;
  // }
  PointCloudcutground->width    = PointCloudcutground->points.size();
  PointCloudcutground->height   = 1;
  PointCloudcutground->is_dense = false;
  PointCloudcutground->points.resize(PointCloudcutground->width * PointCloudcutground->height);
}

void sensor_lidar::PerceptionLidar::ibeoFilter()
{
  pcl::VoxelGrid< pcl::PointXYZ > vg;
  vg.setInputCloud(PointCloudcutground);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  switch (lidar_number)
  {
  case 0:
    PointCloudfilted0->clear();
    vg.filter(*PointCloudfilted0);
    break;
  case 1:
    PointCloudfilted1->clear();
    vg.filter(*PointCloudfilted1);
    break;
  case 2:
    PointCloudfilted2->clear();
    vg.filter(*PointCloudfilted2);
    break;
  case 3:
    PointCloudfilted3->clear();
    vg.filter(*PointCloudfilted3);
    break;
  }
  PointCloudfilted->clear();
  *PointCloudfilted += *PointCloudfilted0;
  *PointCloudfilted += *PointCloudfilted1;
  *PointCloudfilted += *PointCloudfilted2;
  *PointCloudfilted += *PointCloudfilted3;
}

/*
* 当box中心和点云质心相距一定距离,才会认为这是一个特异的点云
* 车辆中心o(0,0) 在质心* 与 边框远端# 形成的坐标系1象限内,
* 远端的判断依据为 边框中心@ 与 质心* 的关系
* // // _________________________#
* // // |
* // // |   *
* // // |           @
* // // |                   o
* // // |
* // // #
*
*/
int judgeSliceArea(pcl::PointXYZ centroid, pcl::PointXYZ min_p, pcl::PointXYZ max_p, float distance_threshold)
{
  int area_ = 0;
  pcl::PointXYZ centre;
  centre.x = 0.5f * (min_p.x + max_p.x);
  centre.y = 0.5f * (min_p.y + max_p.y);

  float distance_ = sqrt(powf(centre.x - centroid.x, 2) + powf(centre.y - centroid.y, 2));

  if (distance_ < distance_threshold) // 不均匀性评价偏差
  {
    area_ = 0;
  }
  else
  {
    if (centre.x < centroid.x && centre.y < centroid.y && 0 < max_p.x && 0 < max_p.y) // 最大点在1象限
    {
      area_ = 1;
    }
    else if (centre.x < centroid.x && centre.y > centroid.y && 0 < max_p.x && 0 > min_p.y) // 2象限
    {
      area_ = 2;
    }
    else if (centre.x > centroid.x && centre.y > centroid.y && 0 > min_p.x && 0 > min_p.y) // 3象限
    {
      area_ = 3;
    }
    else if (centre.x > centroid.x && centre.y < centroid.y && 0 > min_p.x && 0 < max_p.y)
    {
      area_ = 4;
    }
    else
    {
      area_ = 0;
    }
  }
  return area_;
}

/**
 * 通过给定矩形最大/最小点坐标,判断点云是否有点在矩形内部
 * ________________________
 * |                       |
 * |                       |
 * |                       |
 * |_______________________|
 *
 * input:
 *    min_p:AABB框的最小点
 *    max_p:AABB框的最大点
 *    point_cloud:输入点云
 * output:
 *    bool: 有=true,无=false
 * */
bool pointcloudInArea(pcl::PointXYZ min_p, pcl::PointXYZ max_p, pcl::PointCloud< pcl::PointXYZ > &point_cloud)
{
  for (size_t i = 0; i < point_cloud.points.size(); ++i)
  {
    if (point_cloud.points[i].x >= min_p.x && point_cloud.points[i].x <= max_p.x &&
        point_cloud.points[i].y >= min_p.y && point_cloud.points[i].y <= max_p.y)
    {
      return true;
    }
  }
  return false;
}

/**
 * 通过给定矩形最大/最小点坐标,计算有多少点云点在矩形内部
 * input:
 *    min_p:AABB框的最小点
 *    max_p:AABB框的最大点
 *    point_cloud:输入点云
 * output:
 *    size_t: 在点云内部的点的数量
 * */
bool countPointsInArea(pcl::PointXYZ min_p, pcl::PointXYZ max_p, pcl::PointCloud< pcl::PointXYZ > &point_cloud)
{
  size_t number_ = 0;
  for (size_t i = 0; i < point_cloud.points.size(); ++i)
  {
    if (point_cloud.points[i].x >= min_p.x && point_cloud.points[i].x <= max_p.x &&
        point_cloud.points[i].y >= min_p.y && point_cloud.points[i].y <= max_p.y)
    {
      number_++;
    }
  }
  return number_;
}

/**
 * 寻找point_cloud可能的L形双矩形外框交点o
 * _______________________+
 * |_________________o    |
 *                   |    |
 *                   |    |
 * #                 |____|
 *
 * input:
 *    basic_min_p:输入点云AABB框的最小点#
 *    basic_max_p:输入点云AABB框的最大点+
 *    dynamic_direction 移动方向 :
 *      1表示分割点在1象限移动,如上图的o点.
 *      2象限指以(min.x, max.y)为不动点,到(max.x,min.y)之间,以此类推
 *    point_cloud:输入点云
 *    step:寻找矩阵的步长,当两次之间的步长小于该值,则停止迭代
 * output:
 *    slice_point: 双矩形外框交点o pcl::PointXYZ类型
 * theory:
 *    通过判断 #与o(示例)组成的矩形框中是否存在点云点,利用近似牛顿迭代法动态调整o
 * 存在问题:
 *    因为 o的x,y坐标是同时调整的,因此会出现某个方向实际还有很大空间可调整的情况
 * */
pcl::PointXYZ sliceSegment(pcl::PointXYZ &basic_min_p, pcl::PointXYZ &basic_max_p, int dynamic_direction,
                           pcl::PointCloud< pcl::PointXYZ > &point_cloud, float step)
{
  pcl::PointXYZ min_p_, max_p_;
  pcl::PointXYZ slice_p_ = basic_min_p;

  int loopi = 0;
  switch (dynamic_direction)
  {
  case 1: // 1向限,寻找xmax/ymax
    slice_p_.x = basic_min_p.x;
    slice_p_.y = basic_min_p.y;
    min_p_     = basic_min_p;
    max_p_     = basic_max_p;
    while (fabsf(slice_p_.x - max_p_.x) > step && fabsf(slice_p_.y - max_p_.y) > step &&
           loopi < 100) // 步长大于0.1m或者迭代次数小于100
    {
      ++loopi;
      if (pointcloudInArea(min_p_, max_p_, point_cloud)) // 如果有点在矩形框中,那么缩小矩形框
      {
        max_p_.x = (slice_p_.x + max_p_.x) * 0.5f;
        max_p_.y = (slice_p_.y + max_p_.y) * 0.5f;
      }
      else // 保存点,增大矩形框
      {
        slice_p_.x = max_p_.x;
        slice_p_.y = max_p_.y;
        max_p_.x   = (slice_p_.x + basic_max_p.x) * 0.5f;
        max_p_.y   = (slice_p_.y + basic_max_p.y) * 0.5f;
      }
    }
    break;
  case 2: // 2向限,寻找xmax/ymin
    slice_p_.x = basic_min_p.x;
    slice_p_.y = basic_max_p.y;
    min_p_     = basic_min_p;
    max_p_     = basic_max_p;
    while (fabsf(slice_p_.x - max_p_.x) > step && fabsf(slice_p_.y - min_p_.y) > step &&
           loopi < 100) // 步长大于0.1m或者迭代次数小于100
    {
      ++loopi;
      if (pointcloudInArea(min_p_, max_p_, point_cloud)) // 如果有点在矩形框中,那么缩小矩形框
      {
        max_p_.x = (slice_p_.x + max_p_.x) * 0.5f;
        min_p_.y = (slice_p_.y + min_p_.y) * 0.5f;
      }
      else // 保存点,增大矩形框
      {
        slice_p_.x = max_p_.x;
        slice_p_.y = min_p_.y;
        max_p_.x   = (slice_p_.x + basic_max_p.x) * 0.5f;
        min_p_.y   = (slice_p_.y + basic_min_p.y) * 0.5f;
      }
    }
    break;
  case 3: // 3象限,寻找xmin/ymin
    slice_p_.x = basic_max_p.x;
    slice_p_.y = basic_max_p.y;
    min_p_     = basic_min_p;
    max_p_     = basic_max_p;
    while (fabsf(slice_p_.x - min_p_.x) > step && fabsf(slice_p_.y - min_p_.y) > step &&
           loopi < 100) // 步长大于0.1m或者迭代次数小于100
    {
      ++loopi;
      if (pointcloudInArea(min_p_, max_p_, point_cloud)) // 如果有点在矩形框中,那么缩小矩形框
      {
        min_p_.x = (slice_p_.x + min_p_.x) * 0.5f;
        min_p_.y = (slice_p_.y + min_p_.y) * 0.5f;
      }
      else // 保存点,增大矩形框
      {
        slice_p_.x = min_p_.x;
        slice_p_.y = min_p_.y;
        min_p_.x   = (slice_p_.x + basic_min_p.x) * 0.5f;
        min_p_.y   = (slice_p_.y + basic_min_p.y) * 0.5f;
      }
    }
    break;
  case 4: // 4象限,寻找xmin/ymax
    slice_p_.x = basic_max_p.x;
    slice_p_.y = basic_min_p.y;
    min_p_     = basic_min_p;
    max_p_     = basic_max_p;
    while (fabsf(slice_p_.x - min_p_.x) > step && fabsf(slice_p_.y - max_p_.y) > step &&
           loopi < 100) // 步长大于0.1m或者迭代次数小于100
    {
      ++loopi;
      if (pointcloudInArea(min_p_, max_p_, point_cloud)) // 如果有点在矩形框中,那么缩小矩形框
      {
        min_p_.x = (slice_p_.x + min_p_.x) * 0.5f;
        max_p_.y = (slice_p_.y + max_p_.y) * 0.5f;
      }
      else // 保存点,增大矩形框
      {
        slice_p_.x = min_p_.x;
        slice_p_.y = max_p_.y;
        min_p_.x   = (slice_p_.x + basic_min_p.x) * 0.5f;
        max_p_.y   = (slice_p_.y + basic_max_p.y) * 0.5f;
      }
    }
    break;
  default:
    std::cout << ">>>>>> cannot find rectangle <<<<<<<" << std::endl;
    break;
  }
  return slice_p_;
}

#define CELL 0.5
#define XRANGE 100
#define YRANGE 50
#define XSIZE int(XRANGE / CELL) + 1
#define YSIZE int(YRANGE / CELL) + 1
u_int8_t grid_cloud[XSIZE][YSIZE];

void rasterizePointCloud(pcl::PointCloud< pcl::PointXYZ > &in_pointcloud, pcl::PointXYZ &centroid)
{
  size_t x_index_, y_index_;
  for (size_t i = 0; i < XSIZE; ++i)
  {
    for (size_t j = 0; j < YSIZE; ++j)
    {
      grid_cloud[i][j] = 0;
    }
  }
  centroid.x    = 0;
  centroid.y    = 0;
  centroid.z    = 0;
  size_t count_ = 0;

  for (size_t i = 0; i < in_pointcloud.points.size(); ++i)
  {
    x_index_     = (in_pointcloud.points[i].x + XRANGE / 2) / CELL;
    y_index_     = (in_pointcloud.points[i].y + YRANGE / 2) / CELL;
    float range_ = sqrt(pow(in_pointcloud.points[i].x, 2) + pow(in_pointcloud.points[i].y, 2));
    if (x_index_ < 0)
    {
      x_index_ = 0;
    }
    else if (x_index_ >= XSIZE)
    {
      x_index_ = XSIZE - 1;
    }

    if (y_index_ < 0)
    {
      y_index_ = 0;
    }
    else if (y_index_ >= YSIZE)
    {
      y_index_ = YSIZE - 1;
    }
    if (grid_cloud[x_index_][y_index_] == 0)
    {
      grid_cloud[x_index_][y_index_] = range_;
      centroid.x += in_pointcloud.points[i].x;
      centroid.y += in_pointcloud.points[i].y;
      count_ += 1;
    }
  }
  centroid.x /= count_;
  centroid.y /= count_;
}

void sensor_lidar::PerceptionLidar::euclideanCluster()
{
  std::vector< pcl::PointIndices > cluster_indices;
  pcl::search::KdTree< pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree< pcl::PointXYZ >);
  tree->setInputCloud(PointCloudfilted);
  pcl::EuclideanClusterExtraction< pcl::PointXYZ > ec;
  ec.setClusterTolerance(cluster_Tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(PointCloudfilted);
  ec.extract(cluster_indices);

  int id = 0;
  // obstacleFeatureListPointer = new std::list<obstacleFeature >;
  // omp_set_num_threads(omp_get_max_threads());
  // #pragma omp parallel for
  for (int i = 0; i < cluster_indices.size(); i++)
  {
    pcl::PointCloud< pcl::PointXYZ > obstacle_pointcloud_, transformed_pointcloud_;
    pcl::ExtractIndices< pcl::PointXYZ > cluster_filter_;
    pcl::PointXYZ max_p_, min_p_;
    Eigen::Affine3f trans_a3_;

    // 得到原始点云中的障碍物点云
    pcl::PointIndicesPtr inlier_(new pcl::PointIndices(cluster_indices[i]));
    cluster_filter_.setInputCloud(PointCloudfilted);
    cluster_filter_.setIndices(inlier_);
    cluster_filter_.filter(obstacle_pointcloud_);
    // 原始点云下的AABB框
    pcl::getMinMax3D(obstacle_pointcloud_, min_p_, max_p_);

    obstacleFeature oneObstacleFeature;
    oneObstacleFeature.xMax_ = max_p_.x;
    oneObstacleFeature.xMin_ = min_p_.x;
    oneObstacleFeature.yMax_ = max_p_.y;
    oneObstacleFeature.yMin_ = min_p_.y;

    // 将点云旋转到地图坐标系,并沿道路方向排布:目前取固定的33.2705度
    trans_a3_ =
        Eigen::Translation3f(0.0, 0.0, 0.0) * Eigen::AngleAxisf(-(att[0] + 33.2705) * M_PI / 180, Vector3f::UnitZ());
    pcl::transformPointCloud(obstacle_pointcloud_, transformed_pointcloud_, trans_a3_);
    // 地图坐标系下的点云质心
    pcl::PointXYZ transformed_centroid_;
    // pcl::computeCentroid(transformed_pointcloud_, transformed_centroid_);
    rasterizePointCloud(transformed_pointcloud_, transformed_centroid_);
    // 地图坐标系下的AABB框
    pcl::getMinMax3D(transformed_pointcloud_, min_p_, max_p_);
    // 地图坐标系下的AABB框中心
    pcl::PointXYZ box_centre_;
    box_centre_.x = 0.5f * (min_p_.x + max_p_.x);
    box_centre_.y = 0.5f * (min_p_.y + max_p_.y);
    // 判断点云是否是L形,且车体中心在L的凹区域,并返回L角点与车体的相对位置
    int8_t sliced_area_direction_ = judgeSliceArea(transformed_centroid_, min_p_, max_p_, 0.5f);
    // 地图坐标系的旋转回车体坐标系的变换矩阵
    trans_a3_ =
        Eigen::Translation3f(0.0, 0.0, 0.0) * Eigen::AngleAxisf((att[0] + 33.2705) * M_PI / 180, Vector3f::UnitZ());

    // 不是L,或者不在L的凹区域
    if (sliced_area_direction_ == 0)
    {
      pcl::PointXYZ p1_, p2_, p3_, p4_;
      p1_.x = min_p_.x;
      p1_.y = min_p_.y;

      p2_.x = min_p_.x;
      p2_.y = max_p_.y;

      p3_.x = max_p_.x;
      p3_.y = max_p_.y;

      p4_.x = max_p_.x;
      p4_.y = min_p_.y;
      // 将地图坐标系的AABB框数据旋转回车体坐标系
      p1_ = pcl::transformPoint(p1_, trans_a3_);
      p2_ = pcl::transformPoint(p2_, trans_a3_);
      p3_ = pcl::transformPoint(p3_, trans_a3_);
      p4_ = pcl::transformPoint(p4_, trans_a3_);

      oneObstacleFeature.x1_ = p1_.x;
      oneObstacleFeature.y1_ = p1_.y;
      oneObstacleFeature.x2_ = p2_.x;
      oneObstacleFeature.y2_ = p2_.y;
      oneObstacleFeature.x3_ = p3_.x;
      oneObstacleFeature.y3_ = p3_.y;
      oneObstacleFeature.x4_ = p4_.x;
      oneObstacleFeature.y4_ = p4_.y;

      obstacleFeatureListPointer.push_back(oneObstacleFeature);
    }
    else
    {
      obstacleFeature oneObstacleFeature0, oneObstacleFeature1; // 划分为两个目标物
      // 在地图坐标系下寻找两个目标物的矩形框
      pcl::PointXYZ object0_p1_, object0_p2_, object0_p3_, object0_p4_, object0_min_p_, object0_max_p_;
      pcl::PointXYZ object1_p1_, object1_p2_, object1_p3_, object1_p4_, object1_min_p_, object1_max_p_;
      // 寻找可能的矩形框交点()
      pcl::PointXYZ slice_point_ = sliceSegment(min_p_, max_p_, sliced_area_direction_, transformed_pointcloud_, 0.2);
      switch (sliced_area_direction_)
      {
      case 1: // ```|
              //  _______
              //  |_____o|____
              //         |    |
              //         |    |
              //         |    |
              //         |____|
        object0_max_p_.x = max_p_.x;
        // object0_max_p_.x = slice_point_.x;
        object0_max_p_.y = max_p_.y;
        object0_min_p_.x = min_p_.x;
        object0_min_p_.y = slice_point_.y;

        object1_max_p_.x = max_p_.x;
        object1_max_p_.y = max_p_.y;
        // object1_max_p_.y = slice_point_.y;
        object1_min_p_.x = min_p_.x;
        object1_min_p_.y = slice_point_.y;
        break;
      case 2: // __|
        object0_max_p_.x = max_p_.x;
        object0_max_p_.y = max_p_.y;
        object0_min_p_.x = slice_point_.x;
        object0_min_p_.y = min_p_.y;
        // object0_min_p_.y = slice_point_.y;

        object1_max_p_.x = max_p_.x;
        // object1_max_p_.x = slice_point_.x;
        object1_max_p_.y = slice_point_.y;
        object1_min_p_.x = min_p_.x;
        object1_min_p_.y = min_p_.y;
        break;
      case 3: // |____
        object0_max_p_.x = slice_point_.x;
        object0_max_p_.y = max_p_.y;
        object0_min_p_.x = min_p_.x;
        object0_min_p_.y = min_p_.y;
        // object0_min_p_.y = slice_point_.y;

        object1_max_p_.x = max_p_.x;
        object1_max_p_.y = slice_point_.y;
        object1_min_p_.x = min_p_.x;
        // object1_min_p_.x = slice_point_.x;
        object1_min_p_.y = min_p_.y;
        break;
      case 4:
        object0_max_p_.x = max_p_.x;
        object0_max_p_.y = max_p_.y;
        object0_min_p_.x = min_p_.x;
        // object0_min_p_.x = slice_point_.x;
        object0_min_p_.y = slice_point_.y;

        object1_max_p_.x = slice_point_.x;
        object1_max_p_.y = max_p_.y;
        // object1_max_p_.y = slice_point_.y;
        object1_min_p_.x = min_p_.x;
        object1_min_p_.y = min_p_.y;
        break;
      default:
        ROS_INFO("\033[1;32m---->\033[0m Error.");
        break;
      }
      // 车体坐标系下的AABB
      object0_p1_.x = object0_min_p_.x;
      object0_p1_.y = object0_min_p_.y;

      object0_p2_.x = object0_min_p_.x;
      object0_p2_.y = object0_max_p_.y;

      object0_p3_.x = object0_max_p_.x;
      object0_p3_.y = object0_max_p_.y;

      object0_p4_.x = object0_max_p_.x;
      object0_p4_.y = object0_min_p_.y;
      // object1
      object1_p1_.x = object1_min_p_.x;
      object1_p1_.y = object1_min_p_.y;

      object1_p2_.x = object1_min_p_.x;
      object1_p2_.y = object1_max_p_.y;

      object1_p3_.x = object1_max_p_.x;
      object1_p3_.y = object1_max_p_.y;

      object1_p4_.x = object1_max_p_.x;
      object1_p4_.y = object1_min_p_.y;

      // 旋转坐标
      slice_point_ = pcl::transformPoint(slice_point_, trans_a3_);

      object0_p1_ = pcl::transformPoint(object0_p1_, trans_a3_);
      object0_p2_ = pcl::transformPoint(object0_p2_, trans_a3_);
      object0_p3_ = pcl::transformPoint(object0_p3_, trans_a3_);
      object0_p4_ = pcl::transformPoint(object0_p4_, trans_a3_);

      oneObstacleFeature0.x1_ = object0_p1_.x;
      oneObstacleFeature0.y1_ = object0_p1_.y;

      oneObstacleFeature0.x2_ = object0_p2_.x;
      oneObstacleFeature0.y2_ = object0_p2_.y;

      oneObstacleFeature0.x3_ = object0_p3_.x;
      oneObstacleFeature0.y3_ = object0_p3_.y;

      oneObstacleFeature0.x4_ = object0_p4_.x;
      oneObstacleFeature0.y4_ = object0_p4_.y;

      oneObstacleFeature0.xMax_ = oneObstacleFeature.xMax_;
      oneObstacleFeature0.xMin_ = slice_point_.x;
      oneObstacleFeature0.yMax_ = oneObstacleFeature.yMax_;
      oneObstacleFeature0.yMin_ = oneObstacleFeature.yMin_;

      obstacleFeatureListPointer.push_back(oneObstacleFeature0);

      object1_p1_ = pcl::transformPoint(object1_p1_, trans_a3_);
      object1_p2_ = pcl::transformPoint(object1_p2_, trans_a3_);
      object1_p3_ = pcl::transformPoint(object1_p3_, trans_a3_);
      object1_p4_ = pcl::transformPoint(object1_p4_, trans_a3_);

      oneObstacleFeature1.x1_ = object1_p1_.x;
      oneObstacleFeature1.y1_ = object1_p1_.y;

      oneObstacleFeature1.x2_ = object1_p2_.x;
      oneObstacleFeature1.y2_ = object1_p2_.y;

      oneObstacleFeature1.x3_ = object1_p3_.x;
      oneObstacleFeature1.y3_ = object1_p3_.y;

      oneObstacleFeature1.x4_ = object1_p4_.x;
      oneObstacleFeature1.y4_ = object1_p4_.y;

      oneObstacleFeature1.xMax_ = slice_point_.x;
      oneObstacleFeature1.xMin_ = oneObstacleFeature.xMin_;
      oneObstacleFeature1.yMax_ = oneObstacleFeature.yMax_;
      oneObstacleFeature1.yMin_ = oneObstacleFeature.yMin_;

      obstacleFeatureListPointer.push_back(oneObstacleFeature1);
    }
  }

  for (std::list< obstacleFeature >::iterator iterList1 = obstacleFeatureListPointer.begin();
       iterList1 != obstacleFeatureListPointer.end(); iterList1++)
  {
    for (std::list< obstacleFeature >::iterator iterList2 = obstacleFeatureListPointer.begin();
         iterList2 != obstacleFeatureListPointer.end();)
    {
      if (iterList1 != iterList2)
      {
        if (iterList1->xMax_ >= iterList2->xMax_ && iterList1->xMin_ <= iterList2->xMin_ &&
            iterList1->yMax_ >= iterList2->yMax_ && iterList1->yMin_ <= iterList2->yMin_)
        {
          obstacleFeatureListPointer.erase(iterList2++);
        }
        else
        {
          iterList2++;
        }
      }
      else
      {
        iterList2++;
      }
    }
  }
  std::cout << "The number of obstacle is: " << obstacleFeatureListPointer.size() << std::endl;
}

bool sensor_lidar::PerceptionLidar::overlapAGV(const obstacleFeature &oneObstacleFeature)
{
  // std::cout << "overlapAGV start" << std::endl;
  float x_vector[] = {5.0, 5.0, -5.0, -5.0}, y_vector[] = {8.0, -8.0, -8.0, 8.0};
  float x1 = oneObstacleFeature.x1_;
  float y1 = oneObstacleFeature.y1_;
  float x2 = oneObstacleFeature.x2_;
  float y2 = oneObstacleFeature.y2_;
  float x3 = oneObstacleFeature.x3_;
  float y3 = oneObstacleFeature.y3_;
  float x4 = oneObstacleFeature.x4_;
  float y4 = oneObstacleFeature.y4_;
  float A1 = y2 - y1, B1 = x1 - x2, C1 = x2 * y1 - x1 * y2, C2 = x4 * (y1 - y2) + y4 * (x2 - x1);
  float A2 = y4 - y1, B2 = x1 - x4, D1 = x4 * y1 - x1 * y4, D2 = x2 * (y1 - y4) + y2 * (x4 - x1);
  bool in_rectangle = false;
  for (int i = 0; i < 4; i++)
  {
    float x = x_vector[i];
    float y = y_vector[i];
    // in_rectangle = in_rectangle & ( ((y2-y1)*(x-x1)-(x2-x1)*(y-y1))*((y2-y1)*(x-x4)-(x2-x1)*(y-y4)) < 0 );
    // in_rectangle = in_rectangle & ( ((y4-y1)*(x-x1)-(x4-x1)*(y-y1))*((y4-y1)*(x-x2)-(x4-x1)*(y-y2)) < 0 );
    bool in_rectangle1 =
        ((fabs(A1 * x + B1 * y + C1) <= fabs(C1 - C2)) && (fabs(A1 * x + B1 * y + C2) <= fabs(C1 - C2)));
    in_rectangle1 = in_rectangle1 &
                    ((fabs(A2 * x + B2 * y + D1) <= fabs(D1 - D2)) && (fabs(A2 * x + B2 * y + D2) <= fabs(D1 - D2)));
    in_rectangle = in_rectangle | in_rectangle1;
  }
  // std::cout << "overlapAGV end" << std::endl;
  return in_rectangle;
}

bool sensor_lidar::PerceptionLidar::overlapAGV(const float x2_max, const float x2_min, const float y2_max,
                                               const float y2_min)
{
  // std::cout << "overlapAGV1 start" << std::endl;
  // float x_vector[] = {10.0, 10.0, -10.0, -10.0}, y_vector[] = {8.0, -8.0, -8.0, 8.0};
  float x1_max = 10.0, x1_min = -10.0, y1_max = 10.0, y1_min = -10.0;
  // 重叠区域坐标
  float x_max = std::min(x1_max, x2_max);
  float y_max = std::min(y1_max, y2_max);
  float x_min = std::max(x1_min, x2_min);
  float y_min = std::max(y1_min, y2_min);
  if (x_max < x_min || y_max < y_min)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void sensor_lidar::PerceptionLidar::boundingBoxSegmentation(const pcl::PointIndices &cluster_indice, const float alpha,
                                                            const float yMax, const float yMin,
                                                            obstacleFeature &oneObstacleFeature1,
                                                            obstacleFeature &oneObstacleFeature2)
{
  // std::cout << "boundingBoxSegmentation start" << std::endl;
  float interval = 0.1;
  std::vector< float > y_list;
  for (int i = 0; i < 100; i++)
  {
    y_list.push_back(yMin + interval * i);
    y_list.push_back(yMax - interval * i);
  }
  // float y_list[] ={yMin+interval, yMin+interval*2, yMin+interval*3, yMin+interval*4,
  // yMin+interval*5, yMin+interval*6, yMin+interval*7, yMin+interval*8, yMin+interval*9, yMin+interval*10,

  // yMax-interval, yMax-interval*2, yMax-interval*3,  yMax-interval*4, yMax-interval*5, yMax-interval*6,
  // yMax-interval*7, yMax-interval*8, yMax-interval*9, yMax-interval*10};
  std::vector< float > y_vector;
  for (int i = 0; i < y_list.size(); i++)
  {
    if (y_list[i] > yMin && y_list[i] < yMax)
    {
      y_vector.push_back(y_list[i]);
    }
  }
  int num       = y_vector.size();
  float areaMin = FLT_MAX, xMax1, xMin1, yMax1, yMin1, xMax2, xMin2, yMax2, yMin2;
  float xpMax1, xpMin1, ypMax1, ypMin1, xpMax2, xpMin2, ypMax2, ypMin2;
  float xMax_vector_1[num], xMin_vector_1[num], yMax_vector_1[num], yMin_vector_1[num];
  float xMax_vector_2[num], xMin_vector_2[num], yMax_vector_2[num], yMin_vector_2[num];
  float xpMax_vector_1[num], xpMin_vector_1[num], ypMax_vector_1[num], ypMin_vector_1[num];
  float xpMax_vector_2[num], xpMin_vector_2[num], ypMax_vector_2[num], ypMin_vector_2[num];
  for (int i = 0; i < num; i++)
  {
    xMax_vector_1[i]  = -FLT_MAX;
    yMax_vector_1[i]  = -FLT_MAX;
    xMin_vector_1[i]  = FLT_MAX;
    yMin_vector_1[i]  = FLT_MAX;
    xMax_vector_2[i]  = -FLT_MAX;
    yMax_vector_2[i]  = -FLT_MAX;
    xMin_vector_2[i]  = FLT_MAX;
    yMin_vector_2[i]  = FLT_MAX;
    xpMax_vector_1[i] = -FLT_MAX;
    ypMax_vector_1[i] = -FLT_MAX;
    xpMin_vector_1[i] = FLT_MAX;
    ypMin_vector_1[i] = FLT_MAX;
    xpMax_vector_2[i] = -FLT_MAX;
    ypMax_vector_2[i] = -FLT_MAX;
    xpMin_vector_2[i] = FLT_MAX;
    ypMin_vector_2[i] = FLT_MAX;
  }
  for (std::vector< int >::const_iterator pit = cluster_indice.indices.begin(); pit != cluster_indice.indices.end();
       pit++)
  {
    for (int i = 0; i < num; i++)
    {
      float point_x  = PointCloudfilted->points[*pit].x * cos(alpha) + PointCloudfilted->points[*pit].y * sin(alpha);
      float point_y  = -PointCloudfilted->points[*pit].x * sin(alpha) + PointCloudfilted->points[*pit].y * cos(alpha);
      float point_xp = PointCloudfilted->points[*pit].x;
      float point_yp = PointCloudfilted->points[*pit].y;
      if (point_y > y_vector[i])
      {
        // x, y
        if (point_y > yMax_vector_1[i])
        {
          yMax_vector_1[i] = point_y;
        }
        if (point_y < yMin_vector_1[i])
        {
          yMin_vector_1[i] = point_y;
        }
        if (point_x > xMax_vector_1[i])
        {
          xMax_vector_1[i] = point_x;
        }
        if (point_x < xMin_vector_1[i])
        {
          xMin_vector_1[i] = point_x;
        }
        // xp, yp
        if (point_yp > ypMax_vector_1[i])
        {
          ypMax_vector_1[i] = point_yp;
        }
        if (point_yp < ypMin_vector_1[i])
        {
          ypMin_vector_1[i] = point_yp;
        }
        if (point_xp > xpMax_vector_1[i])
        {
          xpMax_vector_1[i] = point_xp;
        }
        if (point_xp < xpMin_vector_1[i])
        {
          xpMin_vector_1[i] = point_xp;
        }
      }
      else
      {
        // x, y
        if (point_y > yMax_vector_2[i])
        {
          yMax_vector_2[i] = point_y;
        }
        if (point_y < yMin_vector_2[i])
        {
          yMin_vector_2[i] = point_y;
        }
        if (point_x > xMax_vector_2[i])
        {
          xMax_vector_2[i] = point_x;
        }
        if (point_x < xMin_vector_2[i])
        {
          xMin_vector_2[i] = point_x;
        }
        // xp, yp
        if (point_yp > ypMax_vector_2[i])
        {
          ypMax_vector_2[i] = point_yp;
        }
        if (point_yp < ypMin_vector_2[i])
        {
          ypMin_vector_2[i] = point_yp;
        }
        if (point_xp > xpMax_vector_2[i])
        {
          xpMax_vector_2[i] = point_xp;
        }
        if (point_xp < xpMin_vector_2[i])
        {
          xpMin_vector_2[i] = point_xp;
        }
      }
    }
  }
  for (int i = 0; i < num; i++)
  {
    float area1 = (xMax_vector_1[i] - xMin_vector_1[i]) * (yMax_vector_1[i] - yMin_vector_1[i]);
    float area2 = (xMax_vector_2[i] - xMin_vector_2[i]) * (yMax_vector_2[i] - yMin_vector_2[i]);
    if (areaMin > area1 + area2)
    {
      xMax1  = xMax_vector_1[i];
      xMin1  = xMin_vector_1[i];
      yMax1  = yMax_vector_1[i];
      yMin1  = yMin_vector_1[i];
      xMax2  = xMax_vector_2[i];
      xMin2  = xMin_vector_2[i];
      yMax2  = yMax_vector_2[i];
      yMin2  = yMin_vector_2[i];
      xpMax1 = xpMax_vector_1[i];
      xpMin1 = xpMin_vector_1[i];
      ypMax1 = ypMax_vector_1[i];
      ypMin1 = ypMin_vector_1[i];
      xpMax2 = xpMax_vector_2[i];
      xpMin2 = xpMin_vector_2[i];
      ypMax2 = ypMax_vector_2[i];
      ypMin2 = ypMin_vector_2[i];
    }
  }
  float min_width = 3;
  if ((xpMax1 - xpMin1) < min_width)
  {
    float xpMax1_ = (xpMax1 + xpMin1) / 2 + min_width / 2;
    float xpMin1_ = (xpMax1 + xpMin1) / 2 - min_width / 2;
    xpMax1        = xpMax1_;
    xpMin1        = xpMin1_;
  }
  if ((ypMax1 - ypMin1) < min_width)
  {
    float ypMax1_ = (ypMax1 + ypMin1) / 2 + min_width / 2;
    float ypMin1_ = (ypMax1 + ypMin1) / 2 - min_width / 2;
    ypMax1        = ypMax1_;
    ypMin1        = ypMin1_;
  }
  if ((xpMax1 - xpMin1) < min_width)
  {
    float xpMax1_ = (xpMax1 + xpMin1) / 2 + min_width / 2;
    float xpMin1_ = (xpMax1 + xpMin1) / 2 - min_width / 2;
    xpMax1        = xpMax1_;
    xpMin1        = xpMin1_;
  }
  if ((ypMax2 - ypMin2) < min_width)
  {
    float ypMax2_ = (ypMax2 + ypMin2) / 2 + min_width / 2;
    float ypMin2_ = (ypMax2 + ypMin2) / 2 - min_width / 2;
    ypMax2        = ypMax2_;
    ypMin2        = ypMin2_;
  }
  oneObstacleFeature1.x1_   = xMax1 * cos(-alpha) + yMax1 * sin(-alpha);
  oneObstacleFeature1.y1_   = -xMax1 * sin(-alpha) + yMax1 * cos(-alpha);
  oneObstacleFeature1.x2_   = xMin1 * cos(-alpha) + yMax1 * sin(-alpha);
  oneObstacleFeature1.y2_   = -xMin1 * sin(-alpha) + yMax1 * cos(-alpha);
  oneObstacleFeature1.x3_   = xMin1 * cos(-alpha) + yMin1 * sin(-alpha);
  oneObstacleFeature1.y3_   = -xMin1 * sin(-alpha) + yMin1 * cos(-alpha);
  oneObstacleFeature1.x4_   = xMax1 * cos(-alpha) + yMin1 * sin(-alpha);
  oneObstacleFeature1.y4_   = -xMax1 * sin(-alpha) + yMin1 * cos(-alpha);
  oneObstacleFeature1.xMax_ = xpMax1;
  oneObstacleFeature1.xMin_ = xpMin1;
  oneObstacleFeature1.yMax_ = ypMax1;
  oneObstacleFeature1.yMin_ = ypMin1;
  oneObstacleFeature2.x1_   = xMax2 * cos(-alpha) + yMax2 * sin(-alpha);
  oneObstacleFeature2.y1_   = -xMax2 * sin(-alpha) + yMax2 * cos(-alpha);
  oneObstacleFeature2.x2_   = xMin2 * cos(-alpha) + yMax2 * sin(-alpha);
  oneObstacleFeature2.y2_   = -xMin2 * sin(-alpha) + yMax2 * cos(-alpha);
  oneObstacleFeature2.x3_   = xMin2 * cos(-alpha) + yMin2 * sin(-alpha);
  oneObstacleFeature2.y3_   = -xMin2 * sin(-alpha) + yMin2 * cos(-alpha);
  oneObstacleFeature2.x4_   = xMax2 * cos(-alpha) + yMin2 * sin(-alpha);
  oneObstacleFeature2.y4_   = -xMax2 * sin(-alpha) + yMin2 * cos(-alpha);
  oneObstacleFeature2.xMax_ = xpMax2;
  oneObstacleFeature2.xMin_ = xpMin2;
  oneObstacleFeature2.yMax_ = ypMax2;
  oneObstacleFeature2.yMin_ = ypMin2;
  // std::cout << "xMax1 = " << xMax1 << std::endl;
  // std::cout << "xMin1 = " << xMin1 << std::endl;
  // std::cout << "yMax1 = " << yMax1 << std::endl;
  // std::cout << "yMin1 = " << yMin1 << std::endl;
  // std::cout << "xMax2 = " << xMax2 << std::endl;
  // std::cout << "xMin2 = " << xMin2<< std::endl;
  // std::cout << "yMax2 = " << yMax2 << std::endl;
  // std::cout << "yMin2 = " << yMin2 << std::endl;
  // std::cout << "1. x1 = " << oneObstacleFeature1.x1_ << std::endl;
  // std::cout << "1. y1 = " << oneObstacleFeature1.y1_ << std::endl;
  // std::cout << "1. x2 = " << oneObstacleFeature1.x2_ << std::endl;
  // std::cout << "1. y2 = " << oneObstacleFeature1.y2_ << std::endl;
  // std::cout << "1. x3 = " << oneObstacleFeature1.x3_ << std::endl;
  // std::cout << "1. y3 = " << oneObstacleFeature1.y3_ << std::endl;
  // std::cout << "1. x4 = " << oneObstacleFeature1.x4_ << std::endl;
  // std::cout << "1. y4 = " << oneObstacleFeature1.y4_ << std::endl;
  // std::cout << "2. x1 = " << oneObstacleFeature2.x1_ << std::endl;
  // std::cout << "2. y1 = " << oneObstacleFeature2.y1_ << std::endl;
  // std::cout << "2. x2 = " << oneObstacleFeature2.x2_ << std::endl;
  // std::cout << "2. y2 = " << oneObstacleFeature2.y2_ << std::endl;
  // std::cout << "2. x3 = " << oneObstacleFeature2.x3_ << std::endl;
  // std::cout << "2. y3 = " << oneObstacleFeature2.y3_ << std::endl;
  // std::cout << "2. x4 = " << oneObstacleFeature2.x4_ << std::endl;
  // std::cout << "2. y4 = " << oneObstacleFeature2.y4_ << std::endl;
  // std::cout << "boundingBoxSegmentation end" << std::endl;
}

void sensor_lidar::PerceptionLidar::updateNewObject(vector< BaseObject * > &obj_list)
{
#ifdef DEBUG_PERCEPTION_FUSION
  cout << "updateNewObject start" << endl;
#endif

  for (int i = 0; i < obj_list.size(); i++)
  {
    global_object_[global_id_] = obj_list[i];
    global_id_++;
  }

#ifdef DEBUG_PERCEPTION_FUSION
  cout << "updateNewObject end" << endl;
#endif
}

void sensor_lidar::PerceptionLidar::updateAssociatedObject(vector< BaseObject * > &new_obj, Eigen::MatrixXd &matrix)
{
#ifdef DEBUG_PERCEPTION_FUSION
  cout << "updateAssociatedObject start" << endl;
#endif

  // matrix [cols]:旧目标, [row]:新目标
  int cols = matrix.cols();
  int rows = matrix.rows();

  map< uint32_t, BaseObject * >::iterator it;
  for (int i = 0; i < rows; i++)
  {
    int j = 0;
    for (it = global_object_.begin(); it != global_object_.end(); it++)
    {
      if (matrix(i, j) == 1)
      {
        // cout << "****************** global_id: " << it->first << " start ****************"  << endl;
        new_obj[i]->update(*(it->second));
        delete new_obj[i];
        // cout << "****************** global_id: " << it->first << " end   ****************"  << endl;
      }
      j++;
    }
  }

#ifdef DEBUG_PERCEPTION_FUSION
  cout << "updateAssociatedObject end" << endl;
#endif
}

void sensor_lidar::PerceptionLidar::updateUnassociatedObject(vector< BaseObject * > &new_obj, Eigen::MatrixXd &matrix)
{
#ifdef DEBUG_PERCEPTION_FUSION
  cout << "updateUnassociatedObject start" << endl;
#endif

  // matrix [col]旧目标:, [row]:新目标
  int cols = matrix.cols();
  int rows = matrix.rows();

  vector< BaseObject * > obj_list;
  map< uint32_t, BaseObject * >::iterator it;

  for (int i = 0; i < rows; i++)
  {
    int j         = 0;
    bool is_match = false;
    for (it = global_object_.begin(); it != global_object_.end(); it++)
    {
      if (matrix(i, j) == 1)
      {
        is_match = true;
      }
      j++;
    }

    if (!is_match)
    {
      obj_list.push_back(new_obj[i]);
    }
  }

  updateNewObject(obj_list);

#ifdef DEBUG_PERCEPTION_FUSION
  cout << "updateUnassociatedObject end" << endl;
#endif
}

void sensor_lidar::PerceptionLidar::publishFusionObject(ros::Time pub_time, bool is_draw)
{
  cout << "********************** publish : start **********************" << endl;

  cout << "total object before erase loss obj = " << global_object_.size() << endl;

  // 声明决策需求的数据结构
  perception_sensor_msgs::ObjectList global_pub_object_list;
  map< uint32_t, BaseObject * >::iterator it;
  for (it = global_object_.begin(); it != global_object_.end();)
  {
    if (fabs(pub_time.toSec() - (it->second)->timestamp_.toSec()) > 0.11)
    {
      delete it->second;
      global_object_.erase(it++);
    }
    else
    {
      it->second->prediction(pub_time);
      it++;
    }
  }
  cout << "total object after erase loss obj = " << global_object_.size() << endl;

  // 在rviz中显示检测结果
  if (is_draw_)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id    = "/odom";
    marker.header.stamp       = ros::Time::now();
    marker.ns                 = "basic_shapes";
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime           = ros::Duration(0.5);
    marker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.z = 0.8;
    marker.color.b = 0;
    marker.color.g = 0;
    marker.color.r = 1;
    marker.color.a = 1;

    for (it = global_object_.begin(); it != global_object_.end();)
    {
      marker.id = it->first;
      geometry_msgs::Pose pose;
      Eigen::VectorXf state = it->second->getState();
      pose.position.x       = state(2);
      pose.position.y       = state(3);
      pose.position.z       = 2;
      marker.text = string("id: ") + to_string(it->first) + string(" vx: ") + to_string(state(4)).substr(0, 5) +
                    string(" vy: ") + to_string(state(5)).substr(0, 5);
      marker.pose = pose;
      pub_rviz_bounding_box_info_.publish(marker);
      it++;
    }

    // 在rviz上显示世界坐标系下bounding box
    pcl::PointCloud< pcl::PointXYZRGB > show_point;
    sensor_msgs::PointCloud2 msg_point;
    sensor_lidar::showResultInRviz(global_object_, show_point, msg_point);
    pub_rviz_bounding_box_.publish(msg_point);
  }

  // 将BaseObject类型转为决策需要的msg类型
  sensor_lidar::outputTypeTransform(global_object_, global_pub_object_list, pub_time);

  cout << "total pub obj = " << global_pub_object_list.object_list.size() << endl;
  cout << "global_id = " << ( int32_t )global_id_ << endl;

  global_pub_object_list.velocity.linear.x = velocity_xyz[0];
  global_pub_object_list.velocity.linear.y = velocity_xyz[1];
  global_pub_object_list.velocity.linear.z = velocity_xyz[2];
  global_pub_object_list.yaw               = att[0];
  global_pub_object_list.pitch             = att[1];
  global_pub_object_list.roll              = att[2];
  global_pub_object_list.pose.x            = veh_llh[0];
  global_pub_object_list.pose.y            = veh_llh[1];
  global_pub_object_list.pose.z            = veh_llh[2];
  pub_obstacle_info_.publish(global_pub_object_list);

  cout << "********************** publish : end **********************" << endl;
}
