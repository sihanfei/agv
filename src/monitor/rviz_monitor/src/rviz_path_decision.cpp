#include "rviz_path_decision.h" //我们自定义类（class）的头文件

namespace superg_agv
{
namespace monitor
{
// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
RvizPathDecisionClass::RvizPathDecisionClass(ros::NodeHandle &nodehandle) : nh_(nodehandle) // 构造函数
{
  ROS_INFO("in class constructor of RvizPathDecisionClass");
  initializeSubscribers(); // 需要通过成员协助函数帮助构建subscriber，在构造函数中做初始化的工作
  initializePublishers();

  // val_to_remember_ = 0.0; //初始化储存数据的变量的值
}

//以下是一个成员协助函数，帮助构建subscriber
void RvizPathDecisionClass::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  plan_path_decision_sub_ =
      nh_.subscribe("/plan/decision_info", 1, &RvizPathDecisionClass::planPathDecisionInfoCallback, this);
}

//与上相同
void RvizPathDecisionClass::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  rviz_path_decision_pub_ = nh_.advertise< visualization_msgs::MarkerArray >("/monitor/rviz_path_decision_info", 10);
}

//大部分的工作都是在回调函数中完成的
void RvizPathDecisionClass::planPathDecisionInfoCallback(const plan_msgs::DecisionInfoConstPtr &msg)
{
  visualization_msgs::MarkerArray markerArray_point;
  visualization_msgs::Marker points;
  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = "planning";
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = 0;
  points.type               = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.6;
  points.scale.y = 0.6;
  // Points color
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;
  points.color.a = 1.0;

  int icount_ = 0;
  for (common_msgs::PathPoint path_data : msg->path_data_REF)
  {
    icount_++;
    if (icount_ > 8)
    {
      icount_ = 0;
      geometry_msgs::Point p;
      p.x = ( double )(path_data.x);
      p.y = ( double )(path_data.y);
      p.z = 0.5;

      points.points.emplace_back(p);
    }
  }
  markerArray_point.markers.emplace_back(points);
  rviz_path_decision_pub_.publish(markerArray_point);

  // visualization_msgs::MarkerArray decision_markerArray_;
  // visualization_msgs::Marker decision_marker_;
  // ROS_INFO("RvizPathDecisionClass path_data_REF size = %d", ( int )msg->path_data_REF.size());
  // int marker_id_ = 1;
  // for (common_msgs::PathData path_data : msg->path_data_REF)
  // {
  //   marker_id_++;

  //   decision_marker_.header.frame_id    = "/odom";
  //   decision_marker_.header.stamp       = ros::Time::now();
  //   decision_marker_.action             = visualization_msgs::Marker::ADD;
  //   decision_marker_.pose.orientation.w = 1.0;

  //   decision_marker_.ns              = "Decision_point";
  //   decision_marker_.id              = marker_id_;
  //   decision_marker_.type            = visualization_msgs::Marker::CUBE;
  //   decision_marker_.scale.x         = 0.5;
  //   decision_marker_.scale.y         = 0.5;
  //   decision_marker_.scale.z         = 0.5;
  //   decision_marker_.color.r         = 1.0;
  //   decision_marker_.color.g         = 0.0;
  //   decision_marker_.color.b         = 0.0;
  //   decision_marker_.color.a         = 1.0;
  //   decision_marker_.pose.position.x = path_data.x.data / 1000.0;
  //   decision_marker_.pose.position.y = path_data.y.data / 1000.0;
  //   decision_marker_.pose.position.z = 0.5;

  //   decision_markerArray_.markers.emplace_back(decision_marker_);

  // }

  // rviz_path_decision_pub_.publish(decision_markerArray_);
}
} // namespace superg_agv
} // namespace monitor