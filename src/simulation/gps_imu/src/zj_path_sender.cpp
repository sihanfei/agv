#include "nav_msgs/Path.h"
#include "novalet_gps/NovatelPosition.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "location_msgs/LocationPoseInfo.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define FRE 100
#define BUF_LEN 10

//#define M_PI acos(-1)

#define OFFSET_84_Y 22942
#define OFFSET_84_X 33436

using namespace std;

struct positionConf
{
  u_int32_t n_gps_sequence_num;
  float lon;
  float lat;
  float height;
  float velocity_x;
  float velocity_y;
  float heading;
  float pitch;
  float gps_seconds;
};

vector< positionConf > route_data;

positionConf prescan_data;

int split(char dst[][80], char *str, const char *spl)
{
  int n        = 0;
  char *result = NULL;
  result       = strtok(str, spl);
  while (result != NULL)
  {
    strcpy(dst[n++], result);
    result = strtok(NULL, spl);
  }
  return n;
}

int math_tip_ = 1;
int mode_tip  = 0;

void recvRouteCallback(const visualization_msgs::MarkerConstPtr &msg)
{
  if (math_tip_ == 1 && mode_tip == 0)
  {
    positionConf read_position = {0};
    for (size_t i = 0; i < msg->points.size(); i++)
    {
      read_position.n_gps_sequence_num = i;
      read_position.lon                = msg->points[i].y;
      read_position.lat                = msg->points[i].x;
      read_position.height             = msg->points[i].z;
      read_position.velocity_x         = 0;
      read_position.velocity_y         = 0;
      read_position.heading            = 0;
      read_position.pitch              = 0;

      route_data.push_back(read_position);
    }
    ROS_INFO("recieve map size is %d", route_data.size());
    math_tip_ = 2;
    //  start_tip_ = 1;
  }
}

void recvPrescanCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (math_tip_ == 1 && mode_tip == 1)
  {
    prescan_data.lat    = msg->pose.pose.position.x;
    prescan_data.lon    = msg->pose.pose.position.y;
    prescan_data.height = msg->pose.pose.position.z;

    double velocity = msg->twist.twist.linear.x;
    double angle    = 0;
    angle           = msg->pose.pose.orientation.x;

    prescan_data.velocity_x = velocity * cos(angle * M_PI / 180);
    prescan_data.velocity_y = velocity * sin(angle * M_PI / 180);
    prescan_data.heading    = msg->twist.twist.angular.x;
    prescan_data.pitch      = velocity; //暂时定义为速度

    prescan_data.n_gps_sequence_num = msg->header.seq;
    prescan_data.gps_seconds        = msg->header.stamp.nsec;

    ROS_INFO("recieve presacn msg %f, angle:%f, heading:%f, speed:%f", prescan_data.gps_seconds, angle,
             prescan_data.heading, prescan_data.pitch);
    ROS_INFO("angle x = %f y = %f z = %f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
             msg->pose.pose.orientation.z);
    ROS_INFO("location x = %f y = %f z = %f", msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->pose.pose.position.z);
    math_tip_ = 2;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zj_sender");

  ros::NodeHandle n;

  //设置GPS消息格式
  ros::Publisher n_gps_pub = n.advertise< novalet_gps::NovatelPosition >("/drivers/gps_imu/novatel_position", 1, true);
  ros::Publisher location_pub = n.advertise< location_msgs::LocationPoseInfo >("/location/pose_info", BUF_LEN, true);
  ros::Publisher loca_point_pub =
      n.advertise< geometry_msgs::PointStamped >("/drivers/gps_imu/loca_point_msg", BUF_LEN, true);
  // RVIZ 车辆可视化控制信息发布者
  ros::Publisher g_gps_pub_ = n.advertise< nav_msgs::Odometry >("/drivers/gps_imu/odom_msg", BUF_LEN, true);

  // for (size_t wait_i = 5; wait_i > 0; wait_i--)
  // {
  //   sleep(1);
  //   ROS_INFO("wait puber start %d",wait_i);
  // }

  ros::Subscriber route_sub_  = n.subscribe("/map/path_msg", 10, recvRouteCallback);
  ros::Subscriber presacn_sub = n.subscribe("/prescan/location_xyz", 10, recvPrescanCallback);

  if (argc == 2)
  {
    mode_tip = atoi(argv[1]);
    if (mode_tip > 1)
    {
      ROS_INFO("PLEASE input right num ( < 2)");
      exit(1);
    }
  }
  else
  {
    ROS_INFO("PLEASE input right num ( 0 ~ 1)");
    exit(1);
  }
  ROS_INFO("run with mode %d", mode_tip);
  //  ros::Publisher p_gps_pub = n.advertise<nav_msgs::Path>("path_msg",1, true);

  //修改循环周期
  ros::Rate loop_rate(FRE);
  int count               = 0;
  uint32_t count_CHC_send = 0;
  ros::Time current_time, last_time;
  last_time                   = ros::Time::now();
  double last_Z               = 0.0;
  uint32_t count_CHC_send_new = 0;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  tf::TransformBroadcaster odom_broadcaster;

  double zj_time      = 0;
  double zj_time_last = -0.05;
  double zj_x;
  double zj_y;
  double zj_z;
  double zj_velocity_x;
  double zj_velocity_y;
  double zj_heading;
  double zj_pitch;

  uint32_t big_num = 0;

  int loop_i   = 0;
  int loop_tip = 0;

  int begin_tip = 0;
  int end_tip   = 0;

  int send_message_tip = 0;

  while (ros::ok())
  {
    ++count;
    ++send_message_tip;

    // if (send_message_tip > 1000/FRE)
    // {
    //   ROS_INFO("reset subscribe");
    //   route_sub_ = n.subscribe("/map/path_msg", 10, recvRouteCallback);
    //   presacn_sub = n.subscribe("/location_xyz", 10, recvPrescanCallback);
    //   send_message_tip = 0;
    // }

    if (math_tip_ == 2)
    {
      if (mode_tip == 0)
      {
        if (loop_i == 0)
        {
          loop_tip = 1;
          begin_tip++;
          ROS_INFO("go back the begin,%d", begin_tip);
          math_tip_ = 1;
          ROS_INFO("clear the route data.", begin_tip);
          route_data.clear();
        }

        if (loop_i == (route_data.size() - 1))
        {
          loop_tip = 2;
          ROS_INFO("arrived the end,%d", end_tip);
        }

        if (loop_tip == 1)
        {
          loop_i++;
        }

        if (loop_tip == 2)
        {
          loop_i--;
        }

        zj_x          = route_data[loop_i].lat;
        zj_y          = route_data[loop_i].lon;
        zj_z          = 0.0;
        zj_velocity_x = 0.0;
        zj_velocity_y = 0.0;
        zj_heading    = 0.0;
        zj_pitch      = 0.0;
      }
      else if (mode_tip == 1)
      {
        zj_x          = prescan_data.lat;
        zj_y          = prescan_data.lon;
        zj_z          = 0.0;
        zj_velocity_x = prescan_data.velocity_x;
        zj_velocity_y = prescan_data.velocity_y;
        zj_heading    = prescan_data.heading;
        zj_pitch      = prescan_data.pitch; //速度

        //发送RVIZ车辆可视化信息

        ros::Time current_time = ros::Time::now();
        // tf::createQuaternionMsgFromRollPitchYaw(0, 0, -(msg->pose.pose.orientation.x-90)*M_PI/180);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-(zj_heading - 90) * M_PI / 180);

        // first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp    = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_link";

        odom_trans.transform.translation.x = zj_x;
        odom_trans.transform.translation.y = zj_y;
        odom_trans.transform.translation.z = zj_z;
        odom_trans.transform.rotation      = odom_quat;

        // send the transform
        odom_broadcaster.sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp    = current_time;
        odom.header.frame_id = "odom";

        // set the position
        odom.pose.pose.position.x  = zj_x;
        odom.pose.pose.position.y  = zj_y;
        odom.pose.pose.position.z  = zj_z;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id        = "base_link";
        odom.twist.twist.linear.x  = zj_velocity_x;
        odom.twist.twist.linear.y  = zj_velocity_y;
        odom.twist.twist.angular.z = -(zj_heading - 90) * M_PI / 180;

        // publish the message
        g_gps_pub_.publish(odom);

        math_tip_ = 1;
      }
      else
      {
      }

      novalet_gps::NovatelPosition n_gps_p;

      n_gps_p.header.stamp                    = ros::Time::now();
      n_gps_p.header.frame_id                 = "odom";
      n_gps_p.novatel_msg_header.sequence_num = count_CHC_send;
      n_gps_p.novatel_msg_header.message_name = "ZJ_T";

      if (big_num == 1)
      {
        n_gps_p.novatel_msg_header.message_name = "ZJ_N";
      }
      if (big_num > 1)
      {
        n_gps_p.novatel_msg_header.message_name = "ZJ_R";
      }

      n_gps_p.novatel_msg_header.gps_week_num = 0;
      n_gps_p.novatel_msg_header.gps_seconds  = zj_time;
      n_gps_p.solution_status                 = 1;
      n_gps_p.position_type                   = 1;
      n_gps_p.lat                             = zj_x;       //纬度
      n_gps_p.lon                             = zj_y;       //经度
      n_gps_p.height                          = zj_z;       //高度
      n_gps_p.undulation                      = zj_heading; //按照郭晗坐标修改 0-0360
      //      n_gps_p.undulation                      = -(zj_heading - 90); //偏航角

      // if (n_gps_p.undulation < -180)
      // {
      //   n_gps_p.undulation = 360 + n_gps_p.undulation;
      // }

      n_gps_p.lat_sigma    = zj_velocity_y; //俯仰角
      n_gps_p.lon_sigma    = zj_velocity_x; //东向速度
      n_gps_p.height_sigma = zj_pitch;      //速度

      n_gps_pub.publish(n_gps_p);
      ROS_INFO("send presacn msg (%f,%f), heading:%f, speed:%f", n_gps_p.lat, n_gps_p.lon, n_gps_p.undulation,
               n_gps_p.height_sigma);
      //      ROS_INFO("loop_i = %d",loop_i);
      send_message_tip = 0;

      location_msgs::LocationPoseInfo n_loc_pose;
      n_loc_pose.header.stamp         = ros::Time::now();
      n_loc_pose.header.frame_id      = "odom";
      n_loc_pose.heart_beat.data      = count;
      n_loc_pose.pose_valid_flag.data = 1;
      n_loc_pose.position.x.data      = u_int32_t(zj_x * 1000); // mm
      n_loc_pose.position.y.data      = u_int32_t(zj_y * 1000); // mm
      n_loc_pose.heading.data         = u_int16_t(zj_heading * 10);
      n_loc_pose.velocity.data        = u_int16_t(zj_pitch * 1000); //速度 mm/s

      ROS_INFO("send loca_msg (%u,%u), heading:%u, speed:%u", n_loc_pose.position.x.data, n_loc_pose.position.y.data,
               n_loc_pose.heading.data, n_loc_pose.velocity.data);

      location_pub.publish(n_loc_pose);

      geometry_msgs::PointStamped psmsg_gps;

      psmsg_gps.header.stamp    = ros::Time::now();
      psmsg_gps.header.frame_id = "odom";

      psmsg_gps.point.x = zj_x;
      psmsg_gps.point.y = zj_y;
      psmsg_gps.point.z = 0;

      loca_point_pub.publish(psmsg_gps);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("shutting down!\n");
  ros::shutdown();

  return 0;
}
