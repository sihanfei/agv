#include "rviz_lcl_planning.h"

namespace superg_agv
{
namespace monitor
{
// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
RvizLclPlanningClass::RvizLclPlanningClass(ros::NodeHandle &nodehandle) : nh_(nodehandle) // 构造函数
{
  ROS_INFO("in class constructor of RvizObstacleSimClass");
  initializeSubscribers(); // 需要通过成员协助函数帮助构建subscriber，在构造函数中做初始化的工作
  initializePublishers();
}

//以下是一个成员协助函数，帮助构建subscriber
void RvizLclPlanningClass::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  route_ref_planning_sub_ =
      nh_.subscribe("/map/decision_info", 1, &RvizLclPlanningClass::mapRefPointInfoCallback, this);
}

//与上相同
void RvizLclPlanningClass::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  rviz_lcl_planning_info_pub_ = nh_.advertise< visualization_msgs::MarkerArray >("/monitor/rviz_lcl_planning_info", 1);
}

//大部分的工作都是在回调函数中完成的
void RvizLclPlanningClass::mapRefPointInfoCallback(const plan_msgs::DecisionInfoConstPtr &msg)
{
  visualization_msgs::MarkerArray markerArray_point;
  visualization_msgs::Marker points;
  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = "lcl_planning";
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = 0;
  points.type               = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.6;
  points.scale.y = 0.6;
  // Points color
  points.color.r = 0.89;
  points.color.g = 0.98;
  points.color.b = 0.20;
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
  rviz_lcl_planning_info_pub_.publish(markerArray_point);

  // visualization_msgs::MarkerArray decision_markerArray_;
  // visualization_msgs::Marker decision_marker_;
  // ROS_INFO("%d", msg->REF_line_INFO.size());
  // int marker_id_ = 1;
  // for (common_msgs::REFPoint path_data : msg->REF_line_INFO)
  // {
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
  //   decision_marker_.pose.position.x = ( double )(path_data.rx.data) / 1000.0;
  //   decision_marker_.pose.position.y = ( double )(path_data.ry.data) / 1000.0;
  //   decision_marker_.pose.position.z = 0.5;

  //   decision_markerArray_.markers.emplace_back(decision_marker_);
  //   marker_id_++;
  // }

  // rviz_lcl_planning_info_pub_.publish(decision_markerArray_);
}
} // namespace superg_agv
} // namespace monitor