#include "ros/ros.h"
#include <ros/console.h>
#include <ros/package.h>

#include "rviz_obstacle_sim.h"
#include "rviz_path_decision.h"
#include "rviz_reference_line.h"
#include "rviz_select_point.h"
#include "rviz_show_map.h"

#include "rviz_lcl_planning.h"

#include <yaml-cpp/yaml.h>

//#include "glog_helper.h"

using namespace std;
using namespace superg_agv::monitor;

#define NODE_NAME "rviz_visualization_pub"
#define URDF_FILE_PATH "/work/superg_agv/src/monitor/rviz_monitor/data/agv.urdf"

struct Ref_File_Info
{
  std::string ref_line_file_name;
  std::string ref_point_file_name;
  std::string topic_name;
  std::string points_color;
};

namespace YAML
{
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template < typename T > void operator>>(const YAML::Node &node, T &i)
{
  i = node.as< T >();
}
} /* YAML */

// now the extraction operators for these types //重载 >> 预算符。。。。
void operator>>(const YAML::Node &node, Ref_File_Info &ref_file_info)
{
  node["ref_line_file_name"] >> ref_file_info.ref_line_file_name;
  node["ref_point_file_name"] >> ref_file_info.ref_point_file_name;
  node["topic_name"] >> ref_file_info.topic_name;
  node["points_color"] >> ref_file_info.points_color;
}

int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is star, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle nh("~config");
  ros::NodeHandle nh1("~config1");

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  char *home_path           = getenv("HOME");
  char urdf_line_name[1024] = {0};
  sprintf(urdf_line_name, "%s%s", home_path, URDF_FILE_PATH);

  // SUPERG_INFO << "urdf_line_name" << home_path << URDF_FILE_PATH;

  std::ifstream in(urdf_line_name);
  std::ostringstream tmp;
  tmp << in.rdbuf();
  std::string robot_description = tmp.str();

  // ROS_INFO("robot_description = %s", robot_description.c_str());
  n.setParam("robot_description", robot_description);

  //////////////////yaml file

  std::string path_ = ros::package::getPath("rviz_monitor");

  std::string file_name = path_ + "/cfg/ref_line.yaml";
  // ROS_INFO_STREAM_NAMED("rviz_visualization_pub", "ROS package path [" << file_name.c_str() << "]");
  ROS_INFO_STREAM("ROS package path [" << file_name.c_str() << "]");
  // ROS_DEBUG("ROS package path [%s]", file_name.c_str());

  YAML::Node doc;
  doc = YAML::LoadFile(file_name);

  //查询最近参考点 依据文件名称
  std::string find_closest_fef_point_file_name = doc["find_closest_fef_point_file_name"].as< string >();

  //地图图片配置相关
  std::string map_file_path     = doc["map_file_path"].as< string >();
  std::string map_img_file_name = doc["map_img_file_name"].as< string >();
  double map_offset_x           = doc["map_offset_x"].as< double >();
  double map_offset_y           = doc["map_offset_y"].as< double >();
  double map_grid_size          = doc["map_grid_size"].as< double >();

  std::vector< std::string > point_color_v_;
  boost::shared_ptr< RvizReferenceLineClass > ref_line_class_[doc["ref_line_list"].size()];

  //地图参考线
  for (unsigned i = 0; i < doc["ref_line_list"].size(); i++)
  {
    Ref_File_Info ref_file_info_;
    doc["ref_line_list"][i] >> ref_file_info_;
    ref_line_class_[i].reset(new RvizReferenceLineClass(n, nh1, ref_file_info_.ref_line_file_name,
                                                        ref_file_info_.ref_point_file_name, ref_file_info_.topic_name));
    point_color_v_.push_back(ref_file_info_.points_color);
  }

  //
  ros::Rate loop_rate(100);

  //建立 地图 场地集装箱模拟
  // supergai::monitor::rviz::RvizShowMapClass aa(n);
  RvizShowMapClass a_(n, map_file_path, map_img_file_name, map_offset_x, map_offset_y, map_grid_size);
  if (a_.isDataOk_)
    a_.rvizMapPub();

  // RvizReferenceLineClass b_(n, nh1, "ref_line.json", "ref_points.json", "/monitor/reference_line");
  //障碍车模拟
  RvizObstacleSimClass c_(n, nh);
  // rviz 选点
  RvizSelectPointClass d_(n, find_closest_fef_point_file_name);
  //局部路径
  RvizPathDecisionClass e_(n);

  //地图参考线
  // RvizReferenceLineClass b2_(n, nh1, "base_map_02_ref_line.json", "base_map_02_ref_points.json",
  //                            "/monitor/reference2_line");

  // lcl 局部路径
  RvizLclPlanningClass f_(n);

  int count = 401;
  while (ros::ok())
  {
    count++;
    if (count > 1000)
    {
      count = 0;

      for (unsigned i = 0; i < doc["ref_line_list"].size(); i++)
      {
        if (ref_line_class_[i]->isDataOk_)
          ref_line_class_[i]->referenceLinePub(point_color_v_[i]);
      }

      // if (b_.isDataOk_)
      //   b_.referenceLinePub("1.0,0.0,0.0");

      // if (b2_.isDataOk_)
      //   b2_.referenceLinePub("0.0,0.0,1.0");
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
