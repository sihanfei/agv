#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "glog_helper.h"
// #include <glog/logging.h>

using namespace std;

#define FRE 100
#define BUF_LEN 10

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //订阅AGV 速度 GPS信息
    agv_gps_sub = n.subscribe("/odom_msg", BUF_LEN, &SubscribeAndPublish::agv_gps_subCallback, this);
    //动态显示数字
    agv_info_pub = n.advertise< visualization_msgs::MarkerArray >("/monitor/agvinfo", 10);
  }

  void agv_gps_subCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    double gps_lat     = msg->pose.pose.position.x; //纬度 84-Y
    double gps_lon     = msg->pose.pose.position.y; //经度 84-X
    double gps_speed_x = msg->twist.twist.linear.x; //速度.换算为m/s
    double gps_speed_y = msg->twist.twist.linear.y; //速度.换算为m/s

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.header.frame_id    = "/odom";
    marker.header.stamp       = ros::Time::now();
    marker.ns                 = "basic_shapes";
    marker.action             = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type               = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.z = 0.2;

    marker.color.a = 1;

    geometry_msgs::Pose pose;
    ostringstream str;

    //车辆位置
    marker.id       = 1;
    marker.color.b  = 1;
    marker.color.g  = 0;
    marker.color.r  = 0;
    pose.position.x = gps_lat;
    pose.position.y = gps_lon;
    pose.position.z = 2;
    str << "gps_lat:" << fixed << setprecision(2) << abs(gps_lat);
    marker.text = str.str();
    marker.pose = pose;
    markerArray.markers.push_back(marker);
    str.str("");

    marker.id       = 2;
    marker.color.b  = 1;
    marker.color.g  = 0;
    marker.color.r  = 0;
    pose.position.x = gps_lat;
    pose.position.y = gps_lon;
    pose.position.z = 1.8;
    str << "gps_lon:" << fixed << setprecision(2) << abs(gps_lon);
    marker.text = str.str();
    marker.pose = pose;
    markerArray.markers.push_back(marker);
    str.str("");

    //车辆速度
    marker.id       = 3;
    marker.color.b  = 0;
    marker.color.g  = 1;
    marker.color.r  = 0;
    pose.position.x = gps_lat;
    pose.position.y = gps_lon;
    pose.position.z = 1.4;
    str << "speed_x:" << fixed << setprecision(2) << abs(gps_speed_x);
    marker.text = str.str();
    marker.pose = pose;
    markerArray.markers.push_back(marker);
    str.str("");

    marker.id       = 4;
    marker.color.b  = 0;
    marker.color.g  = 1;
    marker.color.r  = 0;
    pose.position.x = gps_lat;
    pose.position.y = gps_lon;
    pose.position.z = 1.2;
    str << "speed_y:" << fixed << setprecision(2) << abs(gps_speed_y);
    marker.text = str.str();
    marker.pose = pose;
    markerArray.markers.push_back(marker);
    str.str("");

    //前轴角度
    //   marker.id = 5;
    //   marker.color.b = 0;
    //   marker.color.g = 0;
    //   marker.color.r = 1;
    //   pose.position.x = gps_lat;
    //   pose.position.y = gps_lon;
    //   pose.position.z = 0.8;
    //   str << "alex_angle:" << fixed << setprecision(4) << control_alex_angle;
    //   marker.text = str.str();
    //   marker.pose = pose;
    //   markerArray.markers.push_back(marker);
    //   str.str("");

    // SUPERG_INFO << "markerArray.markers.size() " << markerArray.markers.size();
    agv_info_pub.publish(markerArray);
  }

private:
  ros::NodeHandle n;

  ros::Publisher agv_info_pub;
  ros::Subscriber agv_gps_sub;

}; // End of class SubscribeAndPublish

int main(int argc, char *argv[])
{
  // google::InitGoogleLogging(argv[0]);
  // FLAGS_stderrthreshold  = google::INFO;
  // FLAGS_colorlogtostderr = true;
  // for (int i = 1; i <= 100; i++)
  // {
  //   LOG_IF(INFO, i == 100) << "LOG_IF(INFO,i==100)  google::COUNTER=" << google::COUNTER << "  i=" << i;
  //   LOG_EVERY_N(INFO, 10) << "LOG_EVERY_N(INFO,10)  google::COUNTER=" << google::COUNTER << "  i=" << i;
  //   LOG_IF_EVERY_N(WARNING, (i > 50), 10) << "LOG_IF_EVERY_N(INFO,(i>50),10)  google::COUNTER=" << google::COUNTER
  //                                         << "  i=" << i;
  //   LOG_FIRST_N(ERROR, 5) << "LOG_FIRST_N(INFO,5)  google::COUNTER=" << google::COUNTER << "  i=" << i;
  // }

  // google::ShutdownGoogleLogging();
  GLogHelper gh(argv[0]);

  SUPERG_INFO << "INFO";
  SUPERG_ERROR << "ERROR";
  SUPERG_WARN << "WARN";

  ros::init(argc, argv, "agv_infotext");

  SubscribeAndPublish test;
  // ros::spin();
  ros::MultiThreadedSpinner s(2); //多线程
  ros::spin(s);

  return 0;
}