#ifndef RVIZ_REFERENCE_LINE_H
#define RVIZ_REFERENCE_LINE_H
//常用的成员
#include <fstream>
#include <list>
#include <sstream>

#include "ros/ros.h" //惯例添加

#include "monitor_common_utils.h"

//需要用到的消息类型
#include "rviz_monitor/myconfigConfig.h"
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #define MAP_REF_POINT_CONNECT_PATH "/work/superg_agv/src/data/map_data"
// #define REF_LINE_NAME "/ref_line_map.json"
using namespace std;

// #define REFLINE_DATA_PATH "/work/superg_agv/src/monitor/rviz_monitor/data/map_data"
// #define REFLINE_DATA_NAME "/ref_points.json" //"/ref_line_map.json"
// //#define REFLINE_DATA_EX_NAME ".csv"

#define OFFSET_X 0
#define OFFSET_Y 0
#define GRID_SIZE 1

namespace superg_agv
{
namespace monitor
{

//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class RvizReferenceLineClass
{
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  RvizReferenceLineClass(ros::NodeHandle &nodehandle, ros::NodeHandle &private_nh, string refline_file_name,
                         string refpoint_file_name, string topic_name);
  bool isDataOk_;
  void referenceLinePub(string point_color);
  int lane_id_;

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数
                       //  std::list<reference_point_xyz> xyz_list[];

  // std::map< int, std::vector< std::vector< double > > > ref_line_data;

  const char ch_filenamelist[27] = "abcdefghijklmnopqrstuvwxyz";

  boost::shared_ptr< dynamic_reconfigure::Server< rviz_monitor::myconfigConfig > > srv_;

  //这里变量后面都加了下划线，作用是提醒这变量只能在该类 中被调用
  ros::Publisher reference_point_pub_;

  ros::Publisher reference_title_pub_;

  bool initializeReferenceLineData(string refline_file_name, string refpoint_file_name);
  void initializePublishers(string topic_name);

  int split(char dst[][80], char *str, const char *spl);

  void reflineColorDynamicReconfigcallback(rviz_monitor::myconfigConfig &config, uint32_t level);

  // 20190718
  std::map< int, RefLine > ref_line_map;
  std::map< int, std::vector< RefPoints > > ref_points_map;
  std::vector< RefPoints > ref_points_vec;

  void getRefLineFromFile2Map(std::__cxx11::string &file_name, std::map< int, RefLine > &ref_line_map);
  void ss2RefLine(std::stringstream &line_ss, RefLine &out_ref_line);

  void getRefPointFromFile2Map(std::__cxx11::string &file_name, std::map< int, RefLine > &ref_line_map,
                               std::map< int, std::vector< RefPoints > > &ref_points_map,
                               std::vector< RefPoints > &ref_points_vec);
  void ss2RefPoint(std::stringstream &point_ss, RefPoints &out_ref_point);
};
} // namespace superg_agv
} // namespace monitor
#endif // RVIZ_REFERENCE_LINE_H