#ifndef RVIZ_SELECT_POINT_H
#define RVIZ_SELECT_POINT_H
//常用的成员
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

#include "ros/ros.h" //惯例添加

//需要用到的消息类型
#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"
#include "tf/tf.h"

#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "get_ref_line.h"
//#include "get_map_xy_color.h"
namespace superg_agv
{
namespace monitor
{
using namespace std;

// 现在开始定义一个类，包括构造函数，成员函数和成员变量
//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class RvizSelectPointClass
{
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  RvizSelectPointClass(ros::NodeHandle &nodehandle, string file_name);

  double object_point_x_;
  double object_point_y_;

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数

  //这里变量后面都加了下划线，作用是提醒这变量只能在该类 中被调用
  ros::Publisher select_point_info_pub_;
  ros::Subscriber rviz_clicked_sub_;
  ros::Subscriber route_laneID_array_sub_; //全局规划路段id

  ros::Publisher task_click_pub_;

  ros::Publisher rviz_route_ref_pub_;
  ros::Publisher vms_info_pub_;

  // GetRefLine *get_ref_line;

  boost::shared_ptr< GetRefLine > get_ref_line;
  std::map< int, RefLine > ref_line_map;
  std::map< int, std::vector< RefPoints > > ref_points_map;
  std::vector< RefPoints > ref_points_vec;

  // boost::shared_ptr< FindClosestRefPointClass > find_closest_ref_point_;
  // FindClosestRefPointClass find_closest_ref_point_;
  // GetMapXYcolorClass get_map_xy_color_;

  void initializeSubscribers();
  void initializePublishers();

  void rvizClickedCallback(const geometry_msgs::PointStampedConstPtr &msg); // subscriber回调函数的原型

  void recvRouteLaneIDArrayCallback(const std_msgs::Int64MultiArrayConstPtr &msg);
};
} // namespace monitor
} // namespace superg_agv
#endif // RVIZ_SELECT_POINT_H