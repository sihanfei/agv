#ifndef RVIZ_OBSTACLE_SIM_H
#define RVIZ_OBSTACLE_SIM_H
//常用的成员
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

#include "ros/ros.h" //惯例添加

//需要用到的消息类型
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "rviz_monitor/myconfig1_Config.h"
#include <dynamic_reconfigure/server.h>

using namespace std;
namespace superg_agv
{
namespace monitor
{

// 现在开始定义一个类，包括构造函数，成员函数和成员变量
//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class RvizObstacleSimClass
{
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  RvizObstacleSimClass(ros::NodeHandle &nodehandle, ros::NodeHandle &private_nh);

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数

  //这里变量后面都加了下划线，作用是提醒这变量只能在该类 中被调用
  ros::Publisher rviz_obstacle_info_pub_;
  ros::Subscriber prescan_obstacle_location_sub_;

  boost::shared_ptr< dynamic_reconfigure::Server< rviz_monitor::myconfig1_Config > > srv_;

  void initializeSubscribers();
  void initializePublishers();

  void perscanObstacleCallback(const nav_msgs::OdometryConstPtr &msg); // subscriber回调函数的原型

  void dynamicReconfigcallback(rviz_monitor::myconfig1_Config &config, uint32_t level);
};
} // namespace superg_agv
} // namespace monitor
#endif // RVIZ_OBSTACLE_SIM_H