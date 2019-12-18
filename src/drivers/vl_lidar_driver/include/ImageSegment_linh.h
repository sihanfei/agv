#ifndef IMAGE_SEGMENT_H
#define IMAGE_SEGMENT_H

#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <ros/ros.h>

#include "rawdata.h"

const int N_SCANS = 16; //垂直分辨率2度
// const int HORIZON_SCANS = 1800; //水平分辨率0.2度

//点云分簇类
class ImageSegment
{
public:
  //构造函数和析构函数
  ImageSegment();
  ~ImageSegment();

  //输入原始点云
  void setPointCloud(velodyne_driver::VPointCloud laser);

  void setPacketsSize(int size_);

  void tf(const Eigen::Matrix4f m1);

  //点云分割
  bool segment();

  pcl::PointCloud< pcl::PointXYZI > getGround();
  pcl::PointCloud< pcl::PointXYZI > getFilteredCloud();

private:
  //提取地面
  void detectGround();

  //分簇标记
  void labelComponent(int row, int col);

  //初始化
  void init();

  pcl::PointCloud< pcl::PointXYZI > full_cloud_; //保存原始点云，按照行号排序

  //数据包数量
  int packets_size_;
  int HORIZON_SCANS;

  pcl::PointCloud< pcl::PointXYZI > ground_;         //地面点
  pcl::PointCloud< pcl::PointXYZI > filtered_cloud_; //除去地面点后的点
};

#endif
