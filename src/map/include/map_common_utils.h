#ifndef MAP_INCLUDE_MAP_COMMON_UTILS_H_
#define MAP_INCLUDE_MAP_COMMON_UTILS_H_

#include "ros/ros.h"
#include <string>
#include <vector>

namespace superg_agv
{
namespace map
{
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
  int ref_line_count;
  std::vector< int > ref_line_id_list;
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

struct BoundrayPoint
{
  int boundray_id;
  Point3 point;
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

struct BoundrayLine
{
  int id;
  Point3 begin;
  Point3 end;
};

struct BoundrayCircle
{
  int id;
  Point3 centre;
  double radius;
  double begin_radian;
  double end_radian;
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

} // namespace map
} // namespace superg_agv
#endif