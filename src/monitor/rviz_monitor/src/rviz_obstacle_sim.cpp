#include "rviz_obstacle_sim.h" //我们自定义类（class）的头文件

namespace superg_agv
{
namespace monitor
{
// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
RvizObstacleSimClass::RvizObstacleSimClass(ros::NodeHandle &nodehandle, ros::NodeHandle &private_nh)
    : nh_(nodehandle) // 构造函数
{
  ROS_INFO("in class constructor of RvizObstacleSimClass");
  initializeSubscribers(); // 需要通过成员协助函数帮助构建subscriber，在构造函数中做初始化的工作
  initializePublishers();

  srv_ = boost::make_shared< dynamic_reconfigure::Server< rviz_monitor::myconfig1_Config > >(private_nh);

  dynamic_reconfigure::Server< rviz_monitor::myconfig1_Config >::CallbackType f;

  f = boost::bind(&RvizObstacleSimClass::dynamicReconfigcallback, this, _1, _2);
  srv_->setCallback(f);

  // val_to_remember_ = 0.0; //初始化储存数据的变量的值
}

//以下是一个成员协助函数，帮助构建subscriber
void RvizObstacleSimClass::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  prescan_obstacle_location_sub_ =
      nh_.subscribe("/prescan/obstacle_location", 1, &RvizObstacleSimClass::perscanObstacleCallback, this);
}

//与上相同
void RvizObstacleSimClass::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  rviz_obstacle_info_pub_ = nh_.advertise< visualization_msgs::MarkerArray >("/monitor/rviz_obstacle_info", 10);
}

void RvizObstacleSimClass::dynamicReconfigcallback(rviz_monitor::myconfig1_Config &config, uint32_t level)
{
  ROS_INFO("obstacle_color: %d", config.obstacle_color);
}
//大部分的工作都是在回调函数中完成的
void RvizObstacleSimClass::perscanObstacleCallback(const nav_msgs::OdometryConstPtr &msg)
{

  visualization_msgs::MarkerArray obstacle_markerArray_;
  visualization_msgs::Marker obstacle_marker_;
  ostringstream str_;

  obstacle_marker_.header.frame_id    = "/odom";
  obstacle_marker_.header.stamp       = ros::Time::now();
  obstacle_marker_.action             = visualization_msgs::Marker::ADD;
  obstacle_marker_.pose.orientation.w = 1.0;
  //障碍物形状
  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  obstacle_marker_.ns              = "obstacle_shape";
  obstacle_marker_.id              = 1;
  obstacle_marker_.type            = visualization_msgs::Marker::CUBE;
  obstacle_marker_.scale.x         = msg->twist.twist.linear.x;
  obstacle_marker_.scale.y         = msg->twist.twist.linear.y;
  obstacle_marker_.scale.z         = msg->twist.twist.linear.z;
  obstacle_marker_.color.r         = 1.0;
  obstacle_marker_.color.g         = 1.0;
  obstacle_marker_.color.b         = 0.0;
  obstacle_marker_.color.a         = 1.0;
  obstacle_marker_.pose.position.x = msg->pose.pose.position.x;
  obstacle_marker_.pose.position.y = msg->pose.pose.position.y;
  obstacle_marker_.pose.position.z = msg->twist.twist.linear.z / 2;
  obstacle_marker_.pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(msg->pose.pose.orientation.x - 90) * M_PI / 180);
  obstacle_markerArray_.markers.emplace_back(obstacle_marker_);
  //障碍物ID文本
  str_ << "ID:" << msg->pose.pose.orientation.w;
  obstacle_marker_.text            = str_.str();
  obstacle_marker_.ns              = "obstacle_ID";
  obstacle_marker_.id              = 2;
  obstacle_marker_.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  obstacle_marker_.scale.x         = 1;
  obstacle_marker_.scale.y         = 1;
  obstacle_marker_.scale.z         = 1;
  obstacle_marker_.color.r         = 0.0;
  obstacle_marker_.color.g         = 1.0;
  obstacle_marker_.color.b         = 0.0;
  obstacle_marker_.color.a         = 1.0;
  obstacle_marker_.pose.position.x = msg->pose.pose.position.x;
  obstacle_marker_.pose.position.y = msg->pose.pose.position.y;
  obstacle_marker_.pose.position.z = 5;
  obstacle_markerArray_.markers.emplace_back(obstacle_marker_);
  str_.str("");
  //障碍物速度文本
  str_ << "SPEED:" << msg->twist.twist.angular.x;
  obstacle_marker_.text            = str_.str();
  obstacle_marker_.ns              = "obstacle_speed";
  obstacle_marker_.id              = 3;
  obstacle_marker_.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  obstacle_marker_.scale.x         = 1;
  obstacle_marker_.scale.y         = 1;
  obstacle_marker_.scale.z         = 1;
  obstacle_marker_.color.r         = 0.0;
  obstacle_marker_.color.g         = 1.0;
  obstacle_marker_.color.b         = 0.0;
  obstacle_marker_.color.a         = 1.0;
  obstacle_marker_.pose.position.x = msg->pose.pose.position.x;
  obstacle_marker_.pose.position.y = msg->pose.pose.position.y;
  obstacle_marker_.pose.position.z = 6;
  obstacle_markerArray_.markers.emplace_back(obstacle_marker_);

  //在这里之所以可以使用publishers的功能，是因为在类中publishers也是其他一个成员函数，所以可以被调用
  rviz_obstacle_info_pub_.publish(obstacle_markerArray_);
}
} // namespace superg_agv
} // namespace monitor