#include "coordinate.h"
#include "nav_msgs/Path.h"
#include "novalet_gps/NovatelPosition.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "location_msgs/FusionDataInfo.h"
#include "location_sensor_msgs/FixedLidarInfo.h"
#include "location_sensor_msgs/IMUAndGNSSInfo.h"
#include "location_sensor_msgs/LidarInfo.h"
#include "location_sensor_msgs/UWBInfo.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "control_msgs/AGVRunningStatus.h"
#include "plan_msgs/DecisionInfo.h"

#include <time.h>

#include "std_msgs/String.h"

#define FRE 100
#define BUF_LEN 10

#define Ra 6378137.0           //地球长半轴
#define Rb 6356752.314         //地球短半轴
#define FF 1.0 / 298.257223563 //扁率
//港区原点坐标(需要实际测量)
#define Har_lan0 22.67080000 * M_PI / 180 //  弧度
#define Har_lon0 114.33280000 * M_PI / 180
#define Har_h0 59.537175
#define ex2_ (2 - FF) * FF
#define N_ Ra / sqrt(1.0 - ex2_ * sin(Har_lan0) * sin(Har_lan0))
#define Har_wgsx0 (N_ + Har_h0) * cos(Har_lan0) * cos(Har_lon0)
#define Har_wgsy0 (N_ + Har_h0) * cos(Har_lan0) * sin(Har_lan0)
#define Har_wgsz0 (N_ * (1 - ex2_) + Har_h0) * sin(Har_lan0)

//两种模式
//模式一
//模拟  P2
//模拟  UWB
//模拟  Lidar
//模拟  UWB

//模式二
//模拟  融合定位

using namespace std;
using namespace Eigen;

location_msgs::FusionDataInfo recv_location_data;
int recv_location_data_tip = 0;
uint32_t recv_location_data_num = 0;
plan_msgs::DecisionInfo decision_data;
int decision_recv_tip = 0;
int ref_index = -1;
int max_ref_num = 0;

int mode_tip_1 = 1;
int mode_tip_2 = 0;

int speed = 10; // 1m/s
int math_rate = 0;

double pos_x = 0;
double pos_y = 0;
double pos_heading = 0;

void fusionSend(location_msgs::FusionDataInfo &fusion_data_msg);
void p2Send(location_sensor_msgs::IMUAndGNSSInfo &p2_msg);
void UWBSend();
void lidarSend();
void fixedLidarSend();
void rvizSend();

struct timeDate
{
  uint32_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  float msec;
} timeDate;

void getTime()
{
  time_t tt;
  tm *t = gmtime(&tt);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  timeDate.year = uint32_t(t->tm_year + 1900);
  timeDate.month = uint8_t(t->tm_mon + 1);
  timeDate.day = uint8_t(t->tm_mday);
  timeDate.hour = uint8_t(t->tm_hour);
  timeDate.min = uint8_t(t->tm_min);
  timeDate.sec = uint8_t(t->tm_sec);
  timeDate.msec = float(tv.tv_usec / 1000);
}

void recvPrescanCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (mode_tip_2 == 0)
  {
    recv_location_data.pose.x = msg->pose.pose.position.x;
    recv_location_data.pose.y = msg->pose.pose.position.y;
    recv_location_data.pose.z = msg->pose.pose.position.z;

    recv_location_data.yaw = msg->twist.twist.angular.x;
    recv_location_data.pitch = msg->pose.pose.orientation.y;
    recv_location_data.roll = msg->pose.pose.orientation.z;

    double velocity = msg->twist.twist.linear.x;
    double angle = msg->twist.twist.angular.x;

    recv_location_data.velocity.linear.x = abs(velocity * cos(angle * M_PI / 180));
    recv_location_data.velocity.linear.y = abs(velocity * sin(angle * M_PI / 180));
    recv_location_data.velocity.linear.z = 0;

    getTime();
    recv_location_data.year = timeDate.year;
    recv_location_data.month = timeDate.month;
    recv_location_data.day = timeDate.day;
    recv_location_data.hour = timeDate.hour;
    recv_location_data.min = timeDate.min;
    recv_location_data.sec = timeDate.sec;
    recv_location_data.msec = timeDate.msec;

    recv_location_data.fuse_state = 1;
    recv_location_data.satellite_status = 1;
    recv_location_data.system_status = 1;
    recv_location_data.pose_confidence = 1;

    recv_location_data_tip = 1;
    ++recv_location_data_num;
    ROS_INFO("Recv %u (%lf,%lf,%lf) speed:%lf heading:%lf angle_x:%lf", recv_location_data_num,
             recv_location_data.pose.x, recv_location_data.pose.y, recv_location_data.pose.z, velocity,
             recv_location_data.yaw, msg->pose.pose.orientation.x);
  }
}

void recvP2Callback(const location_sensor_msgs::IMUAndGNSSInfoConstPtr &msg)
{
  if (mode_tip_2 == 1)
  {

    Vector3d llh_;
    Vector3d henu_;

    llh_(0) = msg->pose.x;
    llh_(1) = msg->pose.y;
    llh_(2) = msg->pose.z;

    transFusionLLHtoHarbourENU(llh_, henu_);

    recv_location_data.pose.x = henu_[0];
    recv_location_data.pose.y = henu_[1];
    recv_location_data.pose.z = henu_[2];

    recv_location_data.yaw = msg->yaw;
    recv_location_data.pitch = msg->pitch;
    recv_location_data.roll = msg->roll;

    recv_location_data.velocity.linear.x = msg->velocity.x;
    recv_location_data.velocity.linear.y = msg->velocity.y;
    recv_location_data.velocity.linear.z = msg->velocity.z;

    recv_location_data.year = msg->year;
    recv_location_data.month = msg->month;
    recv_location_data.day = msg->day;
    recv_location_data.hour = msg->hour;
    recv_location_data.min = msg->min;
    recv_location_data.sec = msg->sec;
    recv_location_data.msec = msg->msec;

    recv_location_data.fuse_state = 1;
    recv_location_data.satellite_status = 1;
    recv_location_data.system_status = 1;
    recv_location_data.pose_confidence = 99;

    recv_location_data_tip = 1;
    ++recv_location_data_num;

    ROS_INFO("Recv %u p2 msg (%lf,%lf,%lf) ->  (%lf,%lf,%lf) ", recv_location_data_num, msg->pose.x, msg->pose.y,
             msg->pose.z, recv_location_data.pose.x, recv_location_data.pose.y, recv_location_data.pose.z);
  }
  else
  {
    if ((mode_tip_1 == 5 || mode_tip_1 == 6) && decision_recv_tip == 1)
    // if (mode_tip_1 == 5 && decision_recv_tip == 1)
    {
      if (ref_index != -1 && max_ref_num > 0 && ref_index < max_ref_num)
      {

        ++math_rate;
        if (mode_tip_2 > 10)
        {
          speed = mode_tip_2;
        }

        if (math_rate > 500 / speed)
        {
          math_rate = 0;
          recv_location_data.header.stamp = msg->header.stamp;
          recv_location_data.pose.x = decision_data.path_data_REF.at(ref_index).x;
          recv_location_data.pose.y = decision_data.path_data_REF.at(ref_index).y;
          recv_location_data.pose.z = 0.0;

          recv_location_data.yaw = decision_data.path_data_REF.at(ref_index).theta;
          recv_location_data.pitch = 0.0;
          recv_location_data.roll = 0.0;

          recv_location_data.velocity.linear.x =
              decision_data.path_data_REF.at(ref_index).v * sin(recv_location_data.yaw * M_PI / 180);
          recv_location_data.velocity.linear.y =
              decision_data.path_data_REF.at(ref_index).v * sin(recv_location_data.yaw * M_PI / 180);
          recv_location_data.velocity.linear.z = 0;

          recv_location_data_tip = 1;

          // ROS_INFO("Tra Ref total %d cur %d (%lf,%lf,%lf) heading: %lf speed(%lf,%lf,%lf) ", max_ref_num, ref_index,
          //          recv_location_data.pose.x, recv_location_data.pose.y, recv_location_data.pose.z,
          //          recv_location_data.yaw, recv_location_data.velocity.linear.x,
          //          recv_location_data.velocity.linear.y,
          //          recv_location_data.velocity.linear.z);

          ++recv_location_data_num;
          ++ref_index;
          if (ref_index >= max_ref_num)
          {
            ref_index = 0;
          }
        }
      }
    }
  }
}

void recvPlanRefCallback(const plan_msgs::DecisionInfoConstPtr &msg)
{
  if (mode_tip_1 == 6)
  {
    decision_data.path_data_REF = msg->path_data_REF;
    decision_recv_tip = 1;
    ref_index = 0;
    max_ref_num = decision_data.path_data_REF.empty() ? -1 : static_cast<int>(decision_data.path_data_REF.size());
  }
}

void recvRefCallback(const plan_msgs::DecisionInfoConstPtr &msg)
{
  if (mode_tip_1 == 5)
  {
    decision_data.path_data_REF = msg->path_data_REF;
    decision_recv_tip = 1;
    ref_index = 0;
    max_ref_num = decision_data.path_data_REF.empty() ? -1 : static_cast<int>(decision_data.path_data_REF.size());
  }
}

// RVIZ 车辆可视化控制信息发布者
ros::Publisher g_gps_pub_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pl_sender");
  ros::NodeHandle n;

  sleep(1);

  //参数说明 5 50
  //模式5，后面是速度，接收到组合导航数据和全局规划参考点路径后将点按照参数中的速度以融合定位的数据格式发布出来

  if (argc == 3)
  {
    mode_tip_1 = atoi(argv[1]); // 2 模拟驱动
    mode_tip_2 = atoi(argv[2]);
  }
  else
  {
    if (argc == 6)
    {
      mode_tip_1 = atoi(argv[1]); // 2 模拟驱动
      mode_tip_2 = atoi(argv[2]);
      pos_x = atof(argv[3]);
      pos_y = atof(argv[4]);
      pos_heading = atof(argv[5]);
    }
    else
    {
      mode_tip_1 = 1; // 0fsm测试 1 融合定位模拟 2 p2
      mode_tip_2 = 0; // 1使用p2数据 0使用presacan数据 2使用固定假数据
      pos_x = 194.258;
      pos_y = -34.9142;
      pos_heading = 0.01;
    }
  }

  ROS_INFO("show par: m1=%d m2=%d (%lf,%lf) heading:%lf", mode_tip_1, mode_tip_2, pos_x, pos_y, pos_heading);

  ros::Publisher p2_pub;
  ros::Publisher uwb_pub;
  ros::Publisher lidar_pub;
  ros::Publisher fixed_lidar_pub;
  // mode_tip_1 = 1
  ros::Publisher fusion_data_pub;

  //输出
  // fsm测试
  ros::Publisher fms_test_pub_ = n.advertise<std_msgs::String>("/test", 1, true);
  // mode_tip_1 = 2  drive
  p2_pub = n.advertise<location_sensor_msgs::IMUAndGNSSInfo>("/drivers/can_wr/imu_gnss_msg", 1, true);
  uwb_pub = n.advertise<location_sensor_msgs::UWBInfo>("/drivers/localization/uwb_msg", 1, true);
  lidar_pub = n.advertise<location_sensor_msgs::LidarInfo>("/drivers/localization/lidar_msg", 1, true);
  fixed_lidar_pub =
      n.advertise<location_sensor_msgs::FixedLidarInfo>("/drivers/localization/fixed_lidar_msg", 1, true);
  // mode_tip_1 = 1 融合定位
  fusion_data_pub = n.advertise<location_msgs::FusionDataInfo>("/localization/fusion_msg", 1, true);

  ros::Publisher agv_status_pub = n.advertise<control_msgs::AGVRunningStatus>("/control/agv_running_status", 1, true);

  //输入
  ros::Subscriber presacn_sub = n.subscribe("/prescan/location_xyz", 10, recvPrescanCallback);
  ros::Subscriber p2_sub = n.subscribe("/drivers/can_wr/imu_gnss_msg", 10, recvP2Callback);
  ros::Subscriber ref_sub = n.subscribe("/map/decision_info", 10, recvRefCallback);
  ros::Subscriber plan_ref_sub = n.subscribe("/plan/decision_info", 10, recvPlanRefCallback);
  //修改循环周期
  ros::Rate loop_rate(FRE);

  while (ros::ok())
  {
    if (recv_location_data_tip == 1)
    {
      recv_location_data_tip = 0;
      if (mode_tip_1 == 1)
      {
        location_msgs::FusionDataInfo fusion_data_msg_;
        fusionSend(fusion_data_msg_);
        fusion_data_pub.publish(fusion_data_msg_);
        ROS_INFO("pub fusion data from plsender!");

        control_msgs::AGVRunningStatus agv_status_msg_;
        agv_status_msg_.header.stamp = ros::Time::now();
        agv_status_msg_.fault_status = 0;
        agv_status_msg_.running_status = 1;
        agv_status_msg_.offset_y = 0;
        agv_status_msg_.offset_heading = 0;
        agv_status_msg_.offset_speed = 0;
        agv_status_pub.publish(agv_status_msg_);
        ROS_INFO("pub agv status data from plsender!");
      }

      if (mode_tip_1 == 5)
      {
        location_msgs::FusionDataInfo fusion_data_msg_;
        fusionSend(fusion_data_msg_);
        fusion_data_pub.publish(fusion_data_msg_);
        ROS_INFO("pub fusion data from ref  %d!", ref_index);
      }

      if (mode_tip_1 == 2)
      {
        location_sensor_msgs::IMUAndGNSSInfo p2_msg_;
        p2Send(p2_msg_);
        p2_pub.publish(p2_msg_);
        ROS_INFO("pub p2 data from plsender !");
      }
      // rvizSend();
    }
    else
    {
      if (mode_tip_2 == 2)
      {
        if (mode_tip_1 == 1)
        {
          location_msgs::FusionDataInfo fusion_data_msg_;
          fusionSend(fusion_data_msg_);
          fusion_data_pub.publish(fusion_data_msg_);
          ROS_INFO("pub fusion data from plsender use false data!");
        }

        if (mode_tip_1 == 2)
        {
          location_sensor_msgs::IMUAndGNSSInfo p2_msg_;
          p2Send(p2_msg_);
          p2_pub.publish(p2_msg_);
          ROS_INFO("pub p2 data from plsender use false data!");
        }
        // rvizSend();
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("shutting down!\n");
  ros::shutdown();

  return 0;
}

void p2Send(location_sensor_msgs::IMUAndGNSSInfo &p2_msg)
{

  ros::Time current_time = ros::Time::now();
  //  p2_msg.header.stamp    = current_time;
  p2_msg.GPS_week = 1;
  p2_msg.GPS_sec = 1;
  p2_msg.base_line = 15;
  p2_msg.NSV1 = 15;
  p2_msg.satellite_status = 1; //数据范围：0-9
  p2_msg.system_status = 1;    //数据范围：0-3
  p2_msg.year = recv_location_data.year;
  p2_msg.month = recv_location_data.month;
  p2_msg.day = recv_location_data.day;
  p2_msg.hour = recv_location_data.hour;
  p2_msg.min = recv_location_data.min;
  p2_msg.sec = recv_location_data.sec;
  p2_msg.msec = recv_location_data.msec;

  p2_msg.pose.x = recv_location_data.pose.x;
  p2_msg.pose.y = recv_location_data.pose.y;
  p2_msg.pose.z = recv_location_data.pose.z;
  p2_msg.yaw = recv_location_data.yaw;
  p2_msg.pitch = recv_location_data.pitch;
  p2_msg.roll = recv_location_data.roll;
  p2_msg.velocity.x = recv_location_data.velocity.linear.x;
  p2_msg.velocity.y = recv_location_data.velocity.linear.y;
  p2_msg.velocity.z = recv_location_data.velocity.linear.z;
  p2_msg.accelgyro.linear.x = 0;
  p2_msg.accelgyro.linear.y = 0;
  p2_msg.accelgyro.linear.z = 0;
  p2_msg.accelgyro.angular.x = 0;
  p2_msg.accelgyro.angular.y = 0;
  p2_msg.accelgyro.angular.z = 0;

  if (mode_tip_2 == 2)
  {
    ros::Time current_time = ros::Time::now();
    //  p2_msg.header.stamp    = current_time;
    p2_msg.GPS_week = 1;
    p2_msg.GPS_sec = 1;
    p2_msg.base_line = 15;
    p2_msg.NSV1 = 15;
    p2_msg.satellite_status = 1; //数据范围：0-9
    p2_msg.system_status = 1;    //数据范围：0-3
    p2_msg.year = 2019;
    p2_msg.month = 5;
    p2_msg.day = 20;
    p2_msg.hour = 11;
    p2_msg.min = 39;
    p2_msg.sec = 21;
    p2_msg.msec = 896;
    p2_msg.pose.x = 22.67085676;  //数据范围：-90~+90(如果是在宁波港使用，则为0~+90)
    p2_msg.pose.y = 114.33342561; //数据范围：-180~+180(如果在宁波港使用，则为0~+180)
    p2_msg.pose.z = 59.5033;
    p2_msg.yaw = 95.2;   //数据范围：0~359.9
    p2_msg.pitch = -0.1; //数据范围：-90~+90
    p2_msg.roll = -1.2;  //数据范围：-180~+180
    p2_msg.velocity.x = 1.22;
    p2_msg.velocity.y = -2.15;
    p2_msg.velocity.z = 1.89;
    p2_msg.accelgyro.linear.x = 0.1;
    p2_msg.accelgyro.linear.y = -0.1;
    p2_msg.accelgyro.linear.z = 0.1;
    p2_msg.accelgyro.angular.x = 0.1;
    p2_msg.accelgyro.angular.y = 0.1;
    p2_msg.accelgyro.angular.z = 0.1;

    // printf("pub p2 (%lf,%lf,%lf) %lf\n", p2_msg.pose.x, p2_msg.pose.y, p2_msg.pose.z, 114.33342561);

    ROS_INFO("pub p2 data (%.8f,%.8f,%.8f))!", p2_msg.pose.x, p2_msg.pose.y, p2_msg.pose.z);
  }
}
void UWBSend()
{
}
void lidarSend()
{
}
void fixedLidarSend()
{
}

void fusionSend(location_msgs::FusionDataInfo &fusion_data_msg)
{
  ros::Time current_time = ros::Time::now();

  fusion_data_msg.header.stamp = current_time;
  if (mode_tip_1 == 5)
  {
    fusion_data_msg.header.stamp = recv_location_data.header.stamp;
  }
  fusion_data_msg.year = recv_location_data.year;
  fusion_data_msg.month = recv_location_data.month;
  fusion_data_msg.day = recv_location_data.day;
  fusion_data_msg.hour = recv_location_data.hour;
  fusion_data_msg.min = recv_location_data.min;
  fusion_data_msg.sec = recv_location_data.sec;
  fusion_data_msg.msec = recv_location_data.msec;
  fusion_data_msg.pose.x = recv_location_data.pose.x;
  fusion_data_msg.pose.y = recv_location_data.pose.y;
  fusion_data_msg.pose.z = recv_location_data.pose.z;

  // fusion_data_msg.pose.x = 43.466027;
  // fusion_data_msg.pose.y = 5.2953002;
  // fusion_data_msg.pose.z = 1.400000;

  fusion_data_msg.yaw = recv_location_data.yaw;
  fusion_data_msg.pitch = recv_location_data.pitch;
  fusion_data_msg.roll = recv_location_data.roll;
  fusion_data_msg.velocity.linear.x = recv_location_data.velocity.linear.x;
  fusion_data_msg.velocity.linear.y = recv_location_data.velocity.linear.y;
  fusion_data_msg.velocity.linear.z = recv_location_data.velocity.linear.z;
  fusion_data_msg.satellite_status = recv_location_data.satellite_status;
  fusion_data_msg.system_status = recv_location_data.system_status;
  fusion_data_msg.fuse_state = recv_location_data.fuse_state;
  fusion_data_msg.pose_confidence = recv_location_data.pose_confidence;

  if (mode_tip_2 == 2)
  {
    ros::Time current_time = ros::Time::now();
    fusion_data_msg.header.stamp = current_time;
    fusion_data_msg.year = 2019;
    fusion_data_msg.month = 5;
    fusion_data_msg.day = 20;
    fusion_data_msg.hour = 11;
    fusion_data_msg.min = 39;
    fusion_data_msg.sec = 21;
    fusion_data_msg.msec = 896;
    fusion_data_msg.satellite_status = 1; //数据范围：0-9
    fusion_data_msg.system_status = 1;    //数据范围：0-3
    fusion_data_msg.pose_confidence = 0.66;
    fusion_data_msg.pose.x = pos_x;
    fusion_data_msg.pose.y = pos_y;
    fusion_data_msg.pose.z = 1.400000;
    fusion_data_msg.pose_llh.x = 43.466027; //数据范围：-90~+90(如果是在宁波港使用，则为0~+90)
    fusion_data_msg.pose_llh.y = 5.295300;  //数据范围：-180~+180(如果在宁波港使用，则为0~+180)
    fusion_data_msg.pose_llh.z = 1.400000;
    fusion_data_msg.yaw = pos_heading; //数据范围：0~359.9
    fusion_data_msg.pitch = -0.1;      //数据范围：-90~+90
    fusion_data_msg.roll = -1.2;       //数据范围：-180~+180
    fusion_data_msg.velocity.linear.x = 1.22;
    fusion_data_msg.velocity.linear.y = -2.15;
    fusion_data_msg.velocity.linear.z = 1.89;
    fusion_data_msg.velocity.angular.x = 0.1;
    fusion_data_msg.velocity.angular.y = 0.1;
    fusion_data_msg.velocity.angular.z = 0.1;
    fusion_data_msg.accel.linear.x = 0.1;
    fusion_data_msg.accel.linear.y = -0.1;
    fusion_data_msg.accel.linear.z = 0.1;
    fusion_data_msg.fuse_state = 1; //数据范围：1-13
    /* code for True */
    ROS_INFO("pub fusion_data_msg data (%lf,%lf,%lf))!", fusion_data_msg.pose_llh.x, fusion_data_msg.pose_llh.y,
             fusion_data_msg.pose_llh.z);
  }
}
void rvizSend()
{

  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time = ros::Time::now();

  double heading_ = recv_location_data.yaw;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-(heading_ - 90) * M_PI / 180);

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = recv_location_data.pose.x;
  odom_trans.transform.translation.y = recv_location_data.pose.y;
  odom_trans.transform.translation.z = recv_location_data.pose.z;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = recv_location_data.pose.x;
  odom.pose.pose.position.y = recv_location_data.pose.y;
  odom.pose.pose.position.z = recv_location_data.pose.z;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = recv_location_data.velocity.linear.x;
  odom.twist.twist.linear.y = recv_location_data.velocity.linear.y;

  odom.twist.twist.angular.z = -(heading_ - 90) * M_PI / 180;

  // publish the message
  g_gps_pub_.publish(odom);
}