#ifndef RVIZ_PATH_DECISION_H
#define RVIZ_PATH_DECISION_H
//常用的成员
#include <fstream>
#include <list>
#include <sstream>

#include "ros/ros.h" //惯例添加

//需要用到的消息类型
#include <plan_msgs/DecisionInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

namespace superg_agv {
namespace monitor {

//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class RvizPathDecisionClass {
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  RvizPathDecisionClass(ros::NodeHandle &nodehandle);

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数

  ros::Subscriber plan_path_decision_sub_; //局部路径规划 轨迹信息订阅

  ros::Publisher rviz_path_decision_pub_; //局部路径规划 RVIZ可视化显示 发布

  void planPathDecisionInfoCallback(const plan_msgs::DecisionInfoConstPtr &msg);

  void initializeSubscribers();
  void initializePublishers();
};
} // namespace superg_agv
} // namespace monitor
#endif // RVIZ_PATH_DECISION_H