#include "ImageSegment_linh.h"
#include <iostream>

//构造函数
ImageSegment::ImageSegment()
{
}

//析构函数
ImageSegment::~ImageSegment()
{
}

void ImageSegment::setPacketsSize(int size_)
{
  packets_size_ = size_;
  HORIZON_SCANS = 24 * size_;
  // std::cout << HORIZON_SCANS << std::endl;
}

void ImageSegment::tf(const Eigen::Matrix4f m1)
{
  pcl::transformPointCloud(full_cloud_, full_cloud_, m1);
}
//输入原始点云
void ImageSegment::setPointCloud(velodyne_driver::VPointCloud laser)
{
  init(); //第一步做初始化

  //构建range_mat_ 和 full_cloud_
  for (velodyne_driver::VPoint point1 : laser)
  {
    // velodyne_driver::VPoint
    // std::cout << point1 << std::endl;
    pcl::PointXYZI point;
    point.x = point1.x;
    point.y = point1.y;
    point.z = point1.z;
    // point.intensity = point1.intensity;

    // std::cout << point << std::endl;

    //计算行下标
    float vertical_angle = round(pcl::rad2deg(atan2(point.z, sqrt(pow(point.x, 2) + pow(point.y, 2))))); //垂直夹角
    int row_ind          = (vertical_angle + N_SCANS - 1) / 2; //计算行下标
    if (row_ind < 0 || row_ind >= N_SCANS)
      continue;

    //计算列下标
    float horizon_angle = pcl::rad2deg(atan2(point.y, point.x)); //水平夹角

    // std::cout << horizon_angle << std::endl;

    //车体点
    // float angle = horizon_angle + 180.0;
    // if (angle > 180.0)
    //   angle -= 360.0;
    // if (angle > min_angle_ && angle < max_angle_)
    //   continue;

    int column_ind = -round((horizon_angle - 90.0) / (360.0 / HORIZON_SCANS)) + HORIZON_SCANS / 2; //计算列下标

    // std::cout << column_ind << std::endl;

    if (column_ind >= HORIZON_SCANS)
      column_ind -= HORIZON_SCANS;
    if (column_ind < 0 || column_ind >= HORIZON_SCANS)
      continue;

    int index       = column_ind + row_ind * HORIZON_SCANS; //按行保存
    point.intensity = row_ind; // intensity整数部分表示线号，小数部分表示是否为遮挡产生的点

    // std::cout << point << std::endl;

    full_cloud_.points[index] = point;
  }
}

//检测地面
void ImageSegment::detectGround()
{

  //假设只有下8线能扫到地面
  for (int j = 0; j < HORIZON_SCANS; j++)
  {
    for (int i = 0; i < N_SCANS / 2; i++)
    {
      //相邻线同一水平角度的两个点下标
      int lower_ind = j + i * HORIZON_SCANS;
      int upper_ind = j + (i + 1) * HORIZON_SCANS;

      // intensity = -1 表示该角度没有扫描到点
      if (full_cloud_.points[lower_ind].intensity == -1 || full_cloud_.points[upper_ind].intensity == -1)
      {
        continue;
      }

      //计算上下相邻两个点的三轴距离差
      float diff_x = full_cloud_.points[upper_ind].x - full_cloud_.points[lower_ind].x;
      float diff_y = full_cloud_.points[upper_ind].y - full_cloud_.points[lower_ind].y;
      float diff_z = full_cloud_.points[upper_ind].z - full_cloud_.points[lower_ind].z;
      //计算夹角，夹角过小表示地面点
      float angle = pcl::rad2deg(atan2(diff_z, sqrt(pow(diff_x, 2) + pow(diff_y, 2))));
      if (fabs(angle) <= 15.0)
      {
        ground_.push_back(full_cloud_.points[lower_ind]);
        ground_.push_back(full_cloud_.points[upper_ind]);
      }
      else
      {
        filtered_cloud_.push_back(full_cloud_.points[lower_ind]);
        filtered_cloud_.push_back(full_cloud_.points[upper_ind]);
      }
    }

    for (int i = N_SCANS / 2; i < N_SCANS; i++)
    {
      int ind = j + i * HORIZON_SCANS;
      if (full_cloud_.points[ind].intensity == -1)
        continue;
      filtered_cloud_.push_back(full_cloud_.points[ind]);
    }
  }
}

//点云分割
bool ImageSegment::segment()
{
  //地面检测
  detectGround();
}

//初始化
void ImageSegment::init()
{
  //初始化full_cloud_用NAN
  full_cloud_.resize(N_SCANS * HORIZON_SCANS);
  pcl::PointXYZI nan_point;
  nan_point.x         = std::numeric_limits< float >::quiet_NaN();
  nan_point.y         = std::numeric_limits< float >::quiet_NaN();
  nan_point.z         = std::numeric_limits< float >::quiet_NaN();
  nan_point.intensity = -1;
  std::fill(full_cloud_.begin(), full_cloud_.end(), nan_point);
  // full_cloud_.clear();
  ground_.clear();
  filtered_cloud_.clear();
}

pcl::PointCloud< pcl::PointXYZI > ImageSegment::getGround()
{
  return ground_;
}

pcl::PointCloud< pcl::PointXYZI > ImageSegment::getFilteredCloud()
{
  return filtered_cloud_;
}
