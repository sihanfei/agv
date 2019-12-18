#ifndef FIND_CLOSEST_REF_POINT_H
#define FIND_CLOSEST_REF_POINT_H
//常用的成员
#include <fstream>
#include <list>
#include <sstream>

#include "ros/ros.h" //惯例添加

#include "monitor_common_utils.h"

namespace superg_agv
{
namespace monitor
{
// #define REF_DATA_PATH "/work/superg_agv/src/data/zhenjiang_Map"
// #define REF_DATA_NAME "/xyz_"
// #define REF_DATA_EX_NAME ".csv"

#define MAP_REF_POINT_CONNECT_PATH "/work/superg_agv/src/routing/map_new/data/"
//#define REF_LINE_NAME "/ref_points.json"

#define OFFSET_X 0
#define OFFSET_Y 0
#define GRID_SIZE 1

#define MAX_DISTANCE_SQUARE 12 * 12

using namespace std;

//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class FindClosestRefPointClass
{
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  FindClosestRefPointClass(string file_name);
  bool isDataOk_;
  int findClosestRefPoint(double in_x, double in_y, double &out_x, double &out_y);

  int findLaneIDWithXY(const double &x_, const double &y_);
  int findLaneRefFromCSV(const int &l_id_, std::vector< RefLaneData > &route_rfd);

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数

  int csv_file_num = 13;

  vector< mapdata > map_ref_line_data_;

  std::map< int, std::vector< std::vector< double > > > ref_line_data;

  bool initializeReferenceLineData();

  void readRefFromCSV(string file_name);

  static bool sortFun(const MinRefPoint &d1, const MinRefPoint &d2);
  int mathDistance(const double &x_, const double &y_, const std::vector< std::vector< double > > &ref_line_point_,
                   MinRefPoint &min_rfd);
  float getCross(const Point &p1, const Point &p2, const Point &p);
  bool isPointInMatrix(const MinRefPoint &rfd, const double &x_, const double &y_);

  double pointDistanceSquare(const double x_1_, const double y_1_, const double x_2_, const double y_2_);
  void vectorToRefLaneData(const std::vector< double > &ref_lp_data_, RefLaneData &rfd_);
  //  double pointDistanceSquare(double in_x, double in_y, mapdata &md_b);

  int split(char dst[][80], char *str, const char *spl);
};
} // namespace monitor
} // namespace superg_agv
#endif // FIND_CLOSEST_REF_POINT_H