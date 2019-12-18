#ifndef RVIZ_LCL_PLANNING_H
#define RVIZ_LCL_PLANNING_H
//常用的成员

//惯例添加
#include "ros/ros.h"

//需要用到的消息类型
#include "plan_msgs/DecisionInfo.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
namespace superg_agv
{
namespace monitor
{

// 现在开始定义一个类，包括构造函数，成员函数和成员变量
//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class RvizLclPlanningClass
{
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  RvizLclPlanningClass(ros::NodeHandle &nodehandle);

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数

  //这里变量后面都加了下划线，作用是提醒这变量只能在该类 中被调用
  ros::Publisher rviz_lcl_planning_info_pub_;
  ros::Subscriber route_ref_planning_sub_;

  void initializeSubscribers();
  void initializePublishers();

  void mapRefPointInfoCallback(const plan_msgs::DecisionInfoConstPtr &msg); // subscriber回调函数的原型
};
} // namespace superg_agv
} // namespace monitor
#endif // RVIZ_LCL_PLANNING_H