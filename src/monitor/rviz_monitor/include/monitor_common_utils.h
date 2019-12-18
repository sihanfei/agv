#ifndef MONITOR_COMMON_UTILS_H_
#define MONITOR_COMMON_UTILS_H_

#include "ros/ros.h"

struct CPU_OCCUPY //定义一个cpu occupy的结构体
{
  char name[20];       //定义一个char类型的数组名name有20个元素
  unsigned int user;   //定义一个无符号的int类型的user
  unsigned int nice;   //定义一个无符号的int类型的nice
  unsigned int system; //定义一个无符号的int类型的system
  unsigned int idle;   //定义一个无符号的int类型的idle
};

struct MEM_OCCUPY //定义一个mem occupy的结构体
{
  char name1[20];
  unsigned int MemTotal;
  char name2[20];
  unsigned int MemFree;
  char name3[20];
  unsigned int Buffers;
  char name4[20];
  unsigned int Cached;
  char name5[20];
  unsigned int SwapCached;
};

struct mapdata
{
  double _x;
  double _y;
};

struct reference_point_xyz
{
  int laneID;
  double _x;
  double _y;
};

struct TaskPoint
{
  double x;
  double y;
  double heading;
  double max_speed;
  int laneID;
  double d_from_line_begin;
};

struct RouteData
{
  int vcu_ID;
  int data_status;
  int data_length;
  std::vector< int > data;
};

struct LineWidth
{
  double left;
  double right;
};

struct RefLaneData
{
  int laneID;
  double point_x;
  double point_y;
  double d_from_begin;
  double theta;
  double kappa;
  double dappa;
  double max_speed;
  double left_lane_width;
  double right_lane_width;
  std::vector< LineWidth > line_width;
};

struct MinRefPoint
{
  int laneID;
  double min_distance;
  RefLaneData ref_data;
};

struct Point
{
  double x;
  double y;
};

struct Point3
{
  double x;
  double y;
  double z;
};

struct AppendixAttribute
{
  int appendix_id;
  int appendix_type;
  int corner_count;
  std::vector< Point3 > corner_point;
};

struct RefPoints
{
  int ref_point_id;
  int ref_line_id;
  Point3 point;
  int line_count;
  std::vector< LineWidth > line_width;
  double kappa; // cuv：1/米
  double dappa; // gcuv：1/米^2
  double d_from_line_begin;
  double theta; // 弧度（0-2pi）
};

struct FindPoint
{
  double distance;
  double radian_diff;
  RefPoints ref_point;
};

struct RefLine
{
  int ref_line_id;
  double speed_min;
  double speed_max;
  double high_max;
  double cuv_min;
  int line_count;
  double gradient;
  int line_direction;
  double total_length;
  Point3 ref_begin;
  Point3 ref_end;
  int material;
  int line_area_value;
  int point_count;
  std::vector< int > appendix_id;
  //  std::vector< AppendixAttribute > appendix_vec;
};

struct TaskPoint3D
{
  double x;
  double y;
  double z;
  double heading;
  double max_speed;
  int laneID;
};

struct DistanceRefPoint
{
  int ref_line_id;
  double min_distance;
  struct RefPoints ref_point;
};

struct UltrasonicStatus
{
  u_int32_t id;
  double distance;
  bool status;
};

#endif // MONITOR_COMMON_UTILS_H_