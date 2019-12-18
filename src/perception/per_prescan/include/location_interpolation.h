#ifndef LOCATION_INTERPOLATION_H_
#define LOCATION_INTERPOLATION_H_

#include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <cstring>
// #include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;

namespace superg_agv
{
namespace drivers
{
struct LocationMathInfo
{
  double time;
  double pos_x;
  double pos_y;
  double pos_z;
  double yaw;
  double pitch;
  double roll;
  int updata_tip;
};

//合并bag偏移数组到frame偏移数组
size_t addBagLocationShiftVer2Frame(std::vector<LocationMathInfo> &bag_ver, std::vector<LocationMathInfo> &frame_ver)
{
  frame_ver.insert(frame_ver.end(), bag_ver.begin(), bag_ver.end());
  return frame_ver.size();
}

//合并bag偏移数组到frame偏移数组
size_t addBagPointsShiftVer2Frame(std::vector<std::vector<double> > &bag_ver,
                                  std::vector<std::vector<double> > &frame_ver)
{
  frame_ver.insert(frame_ver.end(), bag_ver.begin(), bag_ver.end());
  return frame_ver.size();
}

double subRadian(const double &minuend, const double &subtracted)
{
  if (minuend < subtracted)
  {
    return (minuend + 2 * M_PI - subtracted);
  }
  else
  {
    return (minuend - subtracted);
  }
}

//坐标转换y-x-z
vector<double> doYXZShift(LocationMathInfo &location_shift_)
{
  vector<double> xyzinfo = { 0.0, 0.0, 0.0 };
  //绕y
  double x1 = cos(location_shift_.roll) * location_shift_.pos_x + sin(location_shift_.roll) * location_shift_.pos_z;
  double y1 = location_shift_.pos_y;
  double z1 = -sin(location_shift_.roll) * location_shift_.pos_x + cos(location_shift_.roll) * location_shift_.pos_z;
  // Pitch 绕x
  double x2 = x1;
  double y2 = cos(location_shift_.pitch) * y1 - sin(location_shift_.pitch) * z1;
  double z2 = sin(location_shift_.pitch) * y1 + cos(location_shift_.pitch) * z1;
  // yaw 绕z
  xyzinfo[0] = cos(location_shift_.yaw) * x2 - sin(location_shift_.yaw) * y2;
  xyzinfo[1] = sin(location_shift_.yaw) * x2 + cos(location_shift_.yaw) * y2;
  xyzinfo[2] = z2;

  return xyzinfo;
}

//输入第一个包定位数据，最新时间，定位系数，点时间序列，返回点定位时间插值后相对第一个定位数据的偏移序列
int mathPointsShift(const LocationMathInfo &location_f, const LocationMathInfo &location_e,
                    const LocationMathInfo &location_c, const std::vector<double> &points_time_vec,
                    std::vector<std::vector<double> > &points_shift_ver)
{
  int count_ = points_time_vec.empty() ? -1 : static_cast<int>(points_time_vec.size());
  if (count_ > 0 && location_c.updata_tip == 1)
  {
    points_shift_ver.clear();

    LocationMathInfo location_base_;
    location_base_.pos_x = location_e.pos_x - location_f.pos_x;
    location_base_.pos_y = location_e.pos_y - location_f.pos_y;
    location_base_.pos_z = location_e.pos_z - location_f.pos_z;
    location_base_.yaw = subRadian(location_e.yaw, location_f.yaw);
    location_base_.pitch = subRadian(location_e.pitch, location_f.pitch);
    location_base_.roll = subRadian(location_e.roll, location_f.roll);

    for (size_t i = 0; i < count_; ++i)
    {
      LocationMathInfo location_temp_;
      location_temp_.time = points_time_vec.at(i);
      double time_delta = location_temp_.time - location_e.time;
      location_temp_.pos_x = location_base_.pos_x + location_c.pos_x * time_delta;
      location_temp_.pos_y = location_base_.pos_y + location_c.pos_y * time_delta;
      location_temp_.pos_z = location_base_.pos_z + location_c.pos_z * time_delta;
      location_temp_.yaw = location_base_.yaw + location_c.yaw * time_delta;
      location_temp_.pitch = location_base_.pitch + location_c.pitch * time_delta;
      location_temp_.roll = location_base_.roll + location_c.roll * time_delta;
      location_temp_.updata_tip = 1;
      std::vector<double> point_shift_temp = doYXZShift(location_temp_);

      points_shift_ver.push_back(point_shift_temp);
    }
    return count_;
  }
  else
  {
    return -1;
  }
}

//输入第一个包定位数据，最新时间，定位系数，点时间序列，返回点定位时间插值后相对第一个定位数据的偏移序列
int mathPointsLocation(const LocationMathInfo &location_f, const LocationMathInfo &location_e,
                       const LocationMathInfo &location_c, const std::vector<double> &points_time_vec,
                       std::vector<LocationMathInfo> &points_location_shift_ver)
{
  int count_ = points_time_vec.empty() ? -1 : static_cast<int>(points_time_vec.size());
  if (count_ > 0 && location_c.updata_tip == 1)
  {
    points_location_shift_ver.clear();

    LocationMathInfo location_base_;
    location_base_.pos_x = location_e.pos_x - location_f.pos_x;
    location_base_.pos_y = location_e.pos_y - location_f.pos_y;
    location_base_.pos_z = location_e.pos_z - location_f.pos_z;
    location_base_.yaw = subRadian(location_e.yaw, location_f.yaw);
    location_base_.pitch = subRadian(location_e.pitch, location_f.pitch);
    location_base_.roll = subRadian(location_e.roll, location_f.roll);

    for (size_t i = 0; i < count_; ++i)
    {
      LocationMathInfo location_temp_;
      location_temp_.time = points_time_vec.at(i);
      double time_delta = location_temp_.time - location_e.time;
      location_temp_.pos_x = location_base_.pos_x + location_c.pos_x * time_delta;
      location_temp_.pos_y = location_base_.pos_y + location_c.pos_y * time_delta;
      location_temp_.pos_z = location_base_.pos_z + location_c.pos_z * time_delta;
      location_temp_.yaw = location_base_.yaw + location_c.yaw * time_delta;
      location_temp_.pitch = location_base_.pitch + location_c.pitch * time_delta;
      location_temp_.roll = location_base_.roll + location_c.roll * time_delta;
      location_temp_.updata_tip = 1;
      points_location_shift_ver.push_back(location_temp_);
    }
    return count_;
  }
  else
  {
    return -1;
  }
}

//输入旧定位数据和新定位数据，返回最新的系数,并更新更新标记
int updateLineLocationCoefficient(const LocationMathInfo &location_b, const LocationMathInfo &location_e,
                                  LocationMathInfo &location_c)
{
  double time_delta = location_e.time - location_b.time;
  if (time_delta == 0)  //输入两个时间相等，返回-1，系数均值为零
  {
    location_c.time = 0;
    location_c.pos_x = 0;
    location_c.pos_y = 0;
    location_c.pos_z = 0;
    location_c.yaw = 0;
    location_c.pitch = 0;
    location_c.roll = 0;
    location_c.updata_tip = 0;
    return -1;
  }
  else
  {
    if (location_e.time > location_c.time)  //新数据时间大于以前系数时间时计算新系数，返回1，否则不计算新系数，返回0
    {
      location_c.time = location_e.time;
      location_c.pos_x = (location_e.pos_x - location_b.pos_x) / time_delta;
      location_c.pos_y = (location_e.pos_y - location_b.pos_y) / time_delta;
      location_c.pos_z = (location_e.pos_z - location_b.pos_z) / time_delta;
      location_c.yaw = (location_e.yaw - location_b.yaw) / time_delta;
      location_c.pitch = (location_e.pitch - location_b.pitch) / time_delta;
      location_c.roll = (location_e.roll - location_b.roll) / time_delta;
      location_c.updata_tip = 1;
      return 1;
    }
    else
    {
      return 0;
    }
  }
}

int doLidarPointsCorrect(pcl::PointCloud<pcl::PointXYZI> &pcl_points_,
                         std::vector<std::vector<double> > &frame_points_shift_)
{
  if (pcl_points_.size() != frame_points_shift_.size())
  {
    return -1;
  }
  else
  {
    for (size_t i = 0; i < pcl_points_.size(); ++i)
    {
      pcl::PointXYZI &pcl_point_temp = pcl_points_.at(i);
      pcl_point_temp.x += frame_points_shift_.at(i).at(0);
      pcl_point_temp.y += frame_points_shift_.at(i).at(1);
      pcl_point_temp.z += frame_points_shift_.at(i).at(2);
    }
    return 1;
  }
}

//坐标转换y-x-z
vector<double> doYXZShift(double x, double y, double z, double Pitch, double Roll, double Yaw)
{
  vector<double> xyzinfo = { 0.0, 0.0, 0.0 };
  //绕y
  double x1 = cos(Roll) * x + sin(Roll) * z;
  double y1 = y;
  double z1 = -sin(Roll) * x + cos(Roll) * z;
  // Pitch 绕x
  double x2 = x1;
  double y2 = cos(Pitch) * y1 - sin(Pitch) * z1;
  double z2 = sin(Pitch) * y1 + cos(Pitch) * z1;
  // yaw 绕z
  xyzinfo[0] = cos(Yaw) * x2 - sin(Yaw) * y2;
  xyzinfo[1] = sin(Yaw) * x2 + cos(Yaw) * y2;
  xyzinfo[2] = z2;

  return xyzinfo;
}

//输入原始包数据的起始时间、该包数据总点数、数据时间间隔，返回按照总点数填充的时间序列,可增加时间延时
//用于调整整包延时正数为向后调整，负数为向前调整
size_t mathLidarPointsTimeVec(double start_time, int point_amount, double point_duration,
                              std::vector<double> &points_time_vec, double time_delay = 0.0)
{
  points_time_vec.clear();
  for (int i = 0; i < point_amount; i++)
  {
    double time_temp = start_time + point_duration * i + time_delay;
    points_time_vec.emplace_back(time_temp);
  }
  return points_time_vec.size();
}

size_t mathLidarPointsTimeVec(double start_time, int point_amount, std::vector<double> &points_time_vec,
                              double time_delay = 0.0, double point_duration = 2.8 / 1000 / 1000,
                              double group_duration = 55.5 / 1000 / 1000, int group_size = 12)
{
  points_time_vec.clear();
  for (int i = 0; i < point_amount; i++)
  {
    double time_temp = start_time + point_duration * (i % group_size) + group_duration * (i / group_size) + time_delay;
    points_time_vec.emplace_back(time_temp);
  }
  return points_time_vec.size();
}

//输入原始包数据中的角度数组、当前角速度、当前设置的角度间隔，返回扩大一倍后的每个角度时间数组
std::vector<double> mathLidarAzimuthTimeVec(std::vector<double> &old_azimuth_vec, double azimuth_rate,
                                            double azimuth_delta)
{
  std::vector<double> azimuth_time_vec;
  for (size_t i = 0; i < old_azimuth_vec.size(); ++i)
  {
    double azimuth_time_temp = (old_azimuth_vec.at(i) - old_azimuth_vec.at(0)) / (azimuth_rate * azimuth_delta);
    azimuth_time_vec.push_back(azimuth_time_temp);
    if (i < old_azimuth_vec.size())
    {
      if (old_azimuth_vec.at(i) > old_azimuth_vec.at(i + 1))
      {
        double azimuth_temp = ((old_azimuth_vec.at(i) + old_azimuth_vec.at(i + 1)) / 2 - old_azimuth_vec.at(0)) /
                              (azimuth_rate * azimuth_delta);
        // if (azimuth_temp > 360.0)
        // {
        //   azimuth_temp = azimuth_temp - 360;
        // }
        azimuth_time_vec.push_back(azimuth_temp);
      }
      else
      {
        double azimuth_temp = (old_azimuth_vec.at(i) + old_azimuth_vec.at(i + 1) + 360.0) / 2;
        if (azimuth_temp > 360.0)
        {
          azimuth_temp = azimuth_temp - 360;
        }
        azimuth_time_vec.push_back(azimuth_temp);
      }
    }
    else
    {
      double azimuth_temp = old_azimuth_vec.at(i) + azimuth_rate;
      azimuth_time_vec.push_back(azimuth_temp);
    }
  }
  return azimuth_time_vec;
}

}  // namespace map
}  // namespace superg_agv
#endif