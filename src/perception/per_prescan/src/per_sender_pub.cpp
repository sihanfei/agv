#include "common_msgs/DetectionInfo.h"
#include "control_msgs/ADControlAGV.h"
#include "location_msgs/FusionDataInfo.h"
#include "perception_msgs/FusionDataInfo.h"
#include "perception_sensor_msgs/ObjectList.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

using std::__cxx11::string;

int mode_tip_1;
int mode_tip_2;

ros::Subscriber ultr_sub;
ros::Subscriber cam_sub;
ros::Subscriber lidar_sub;
ros::Publisher obstacle_cam_lidar_pub;
ros::Publisher agv_control_pub;

void pushDetectionInfo(const double &in_loc_x, const double &in_loc_y, const double &in_length_x_,
                       const double &in_length_y_, common_msgs::DetectionInfo &obj_info_);
void pushPrescanMarker(const float radian_, const int id_, visualization_msgs::Marker &points);
void pushStaticMarker(const double &in_loc_x_, const double &in_loc_y_, const double &in_length_x_,
                      const double &in_length_y_, const int id_, string ns_, visualization_msgs::Marker &points);
void pushAGVMarker(const double x_, const double y_, const double radian_, const int id_,
                   visualization_msgs::Marker &points);
void recvFusionObuCallback(const perception_msgs::FusionDataInfo::ConstPtr &obu_msg);
void recvFusionObuTestCallback(const perception_msgs::FusionDataInfo::ConstPtr &obu_msg);

struct odometryConf
{
  float ID;
  float gps_seconds;
  float lon;
  float lat;
  float height;
  float velocity;
  float angle_x;
  float volume_x;
  float volume_y;
  float volume_z;
};

odometryConf obstacle_data;
int new_odometry_send_tip = 0;

double fusion_location_x       = 0;
double fusion_location_y       = 0;
double fusion_location_heading = 0;
double fusion_speed_x          = 0;
double fusion_speed_y          = 0;
int stop_index                 = 0;

visualization_msgs::MarkerArray maker_obs_temp;
int fusion_tip = 0;
int max_index  = 0;

void recvObstacleCallback(const nav_msgs::OdometryConstPtr &msg)
{
  obstacle_data.gps_seconds = msg->header.stamp.nsec;
  obstacle_data.ID          = msg->pose.pose.orientation.w;
  obstacle_data.lat         = msg->pose.pose.position.x;
  obstacle_data.lon         = msg->pose.pose.position.y;
  obstacle_data.height      = msg->pose.pose.position.z;
  obstacle_data.velocity    = msg->twist.twist.angular.x;
  //  obstacle_data.angle_x     = -(msg->pose.pose.orientation.x - 90); // Y轴方向角度
  obstacle_data.angle_x  = msg->pose.pose.orientation.x; // gh坐标
  obstacle_data.volume_x = msg->twist.twist.linear.x;    //长
  obstacle_data.volume_y = msg->twist.twist.linear.y;    //宽
  obstacle_data.volume_z = msg->twist.twist.linear.z;    //高
  new_odometry_send_tip  = 1;
}

void recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
  fusion_location_x       = location_msg->pose.x;
  fusion_location_y       = location_msg->pose.y;
  fusion_location_heading = location_msg->yaw * M_PI / 180;
  fusion_speed_x          = location_msg->velocity.linear.x;
  fusion_speed_y          = location_msg->velocity.linear.y;
}

void recvObuCallback(const perception_sensor_msgs::ObjectList::ConstPtr &msg)
{
  int obs_num = ( int )(msg->obstacle_num);
  if (obs_num > 0)
  {
    double temp_x_      = 0;
    double temp_y_      = 0;
    double temp_length_ = 0;
    double temp_width_  = 0;

    visualization_msgs::MarkerArray maker_temp;
    maker_temp.markers.clear();
    int id_index = 0;

    if (msg->sensor_index == 2)
    {
      ROS_INFO("recv ultr obs %d", obs_num);

      double radian_ = fusion_location_heading;
      double x_      = fusion_location_x;
      double y_      = fusion_location_y;

      for (int i = 0; i < obs_num; i++)
      {

        id_index++;
        visualization_msgs::Marker points;

        points.header.frame_id    = "/odom";
        points.header.stamp       = ros::Time::now();
        points.ns                 = "/per/ultr_obstacle_fusion";
        points.action             = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id                 = id_index;
        points.type               = visualization_msgs::Marker::LINE_STRIP;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        // Points are green
        points.color.r = 1;
        points.color.g = 1;
        points.color.b = 0;
        points.color.a = 1.0;

        geometry_msgs::Point p;

        // double min_x = abs(msg->object_list.at(i).peek.at(0).x + msg->object_list.at(i).peek.at(2).x) / 2000 -
        //                abs(msg->object_list.at(i).peek.at(0).x - msg->object_list.at(i).peek.at(2).x) / 2000;
        // double max_x = abs(msg->object_list.at(i).peek.at(0).x + msg->object_list.at(i).peek.at(2).x) / 2000 +
        //                abs(msg->object_list.at(i).peek.at(0).x - msg->object_list.at(i).peek.at(2).x) / 2000;

        // double min_y = abs(msg->object_list.at(i).peek.at(0).y + msg->object_list.at(i).peek.at(1).y) / 2000 -
        //                abs(msg->object_list.at(i).peek.at(0).y - msg->object_list.at(i).peek.at(1).y) / 2000;
        // double max_y = abs(msg->object_list.at(i).peek.at(0).y + msg->object_list.at(i).peek.at(1).y) / 2000 +
        //                abs(msg->object_list.at(i).peek.at(0).y - msg->object_list.at(i).peek.at(1).y) / 2000;

        double min_x = 0 - min(msg->object_list.at(i).peek.at(0).x, msg->object_list.at(i).peek.at(2).x);
        double max_x = 0 - max(msg->object_list.at(i).peek.at(0).x, msg->object_list.at(i).peek.at(2).x);
        double min_y = min(msg->object_list.at(i).peek.at(0).y, msg->object_list.at(i).peek.at(1).y);
        double max_y = max(msg->object_list.at(i).peek.at(0).y, msg->object_list.at(i).peek.at(1).y);
        // max_x        = min_x + (max_x - min_x);
        // max_y        = min_y + (max_y - min_y);

        temp_x_ = min_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = min_x;
        temp_y_ = max_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = max_x;
        temp_y_ = max_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = max_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = min_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        maker_temp.markers.push_back(points);
      }

      max_index = max(max_index, id_index);
      if (id_index < max_index)
      {
        for (size_t i = (id_index + 1); i < max_index; i++)
        {
          visualization_msgs::Marker points;

          points.header.frame_id    = "/odom";
          points.header.stamp       = ros::Time::now();
          points.ns                 = "/per/cam_obstacle_fusion";
          points.action             = visualization_msgs::Marker::DELETE;
          points.pose.orientation.w = 1.0;
          points.id                 = i;
          points.type               = visualization_msgs::Marker::LINE_STRIP;

          maker_temp.markers.push_back(points);
        }
      }
    }

    if (msg->sensor_index == 1)
    {
      ROS_INFO("recv cam obs %d", obs_num);

      double radian_ = fusion_location_heading;
      double x_      = fusion_location_x;
      double y_      = fusion_location_y;

      for (int i = 0; i < obs_num; i++)
      {

        id_index++;
        visualization_msgs::Marker points;

        points.header.frame_id    = "/odom";
        points.header.stamp       = ros::Time::now();
        points.ns                 = "/per/cam_obstacle_fusion";
        points.action             = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id                 = id_index;
        points.type               = visualization_msgs::Marker::LINE_STRIP;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        // Points are green
        points.color.r = 1;
        points.color.g = 0;
        points.color.b = 0;
        points.color.a = 1.0;

        geometry_msgs::Point p;

        // double min_x = abs(msg->object_list.at(i).peek.at(0).x + msg->object_list.at(i).peek.at(2).x) / 2000 -
        //                abs(msg->object_list.at(i).peek.at(0).x - msg->object_list.at(i).peek.at(2).x) / 2000;
        // double max_x = abs(msg->object_list.at(i).peek.at(0).x + msg->object_list.at(i).peek.at(2).x) / 2000 +
        //                abs(msg->object_list.at(i).peek.at(0).x - msg->object_list.at(i).peek.at(2).x) / 2000;

        // double min_y = abs(msg->object_list.at(i).peek.at(0).y + msg->object_list.at(i).peek.at(1).y) / 2000 -
        //                abs(msg->object_list.at(i).peek.at(0).y - msg->object_list.at(i).peek.at(1).y) / 2000;
        // double max_y = abs(msg->object_list.at(i).peek.at(0).y + msg->object_list.at(i).peek.at(1).y) / 2000 +
        //                abs(msg->object_list.at(i).peek.at(0).y - msg->object_list.at(i).peek.at(1).y) / 2000;

        double min_x = 0 - min(msg->object_list.at(i).peek.at(0).x, msg->object_list.at(i).peek.at(2).x);
        double max_x = 0 - max(msg->object_list.at(i).peek.at(0).x, msg->object_list.at(i).peek.at(2).x);
        double min_y = min(msg->object_list.at(i).peek.at(0).y, msg->object_list.at(i).peek.at(1).y);
        double max_y = max(msg->object_list.at(i).peek.at(0).y, msg->object_list.at(i).peek.at(1).y);
        // max_x        = min_x + (max_x - min_x);
        // max_y        = min_y + (max_y - min_y);

        temp_x_ = min_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = min_x;
        temp_y_ = max_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = max_x;
        temp_y_ = max_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = max_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = min_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        maker_temp.markers.push_back(points);
      }

      max_index = max(max_index, id_index);
      if (id_index < max_index)
      {
        for (size_t i = (id_index + 1); i < max_index; i++)
        {
          visualization_msgs::Marker points;

          points.header.frame_id    = "/odom";
          points.header.stamp       = ros::Time::now();
          points.ns                 = "/per/cam_obstacle_fusion";
          points.action             = visualization_msgs::Marker::DELETE;
          points.pose.orientation.w = 1.0;
          points.id                 = i;
          points.type               = visualization_msgs::Marker::LINE_STRIP;

          maker_temp.markers.push_back(points);
        }
      }
    }

    if (msg->sensor_index == 0)
    {
      ROS_INFO("recv lidar obs %d", obs_num);

      double radian_ = fusion_location_heading;
      double x_      = fusion_location_x;
      double y_      = fusion_location_y;
      stop_index++;
      for (int i = 0; i < obs_num; i++)
      {
        // temp_x_      = 0 - (msg->object_list.at(i).peek.at(0).x + msg->object_list.at(i).peek.at(2).x);
        // temp_y_      = (msg->object_list.at(i).peek.at(0).y + msg->object_list.at(i).peek.at(1).y);
        // temp_width_  = abs(msg->object_list.at(i).peek.at(0).x - msg->object_list.at(i).peek.at(2).x);
        // temp_length_ = abs(msg->object_list.at(i).peek.at(0).y - msg->object_list.at(i).peek.at(1).y);

        id_index++;
        visualization_msgs::Marker points;

        points.header.frame_id    = "/odom";
        points.header.stamp       = ros::Time::now();
        points.ns                 = "/per/lidar_obstacle_fusion";
        points.action             = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id                 = id_index;
        points.type               = visualization_msgs::Marker::LINE_STRIP;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        // Points are green
        points.color.r = 0;
        points.color.g = 0;
        points.color.b = 0;
        points.color.a = 1.0;

        geometry_msgs::Point p;

        double max_x = 0 - msg->object_list.at(i).state[0];
        double min_y = msg->object_list.at(i).state[1];
        double min_x = 0 - msg->object_list.at(i).state[2];
        double max_y = msg->object_list.at(i).state[3];

        control_msgs::ADControlAGV control_agv_temp;
        //障碍物存在判断
        if (abs(max_x) < 3 || abs(min_x) < 3)
        {
          if (abs(min(min_y, max_y)) < (8 + 15))
          {
            stop_index = 0;
            //减速
            control_agv_temp.Vel_Req = 0;
            control_agv_temp.EStop   = 0;
            double speed_temp        = fusion_speed_x + fusion_speed_y;
            if (abs(min(min_y, max_y)) < (8 + 10) && speed_temp > 0.1)
            {
              //急停
              control_agv_temp.EStop = 1;

              ROS_INFO("set EStop = 1");
            }
            else
            {
              ROS_INFO("set Vel_Req = 0");
            }

            agv_control_pub.publish(control_agv_temp);
          }
        }
        else
        {
          if (stop_index > 10)
          {
            control_agv_temp.Vel_Req = 1;
            control_agv_temp.EStop   = 0;
            agv_control_pub.publish(control_agv_temp);
            ROS_INFO("go again ---------> target");
          }
        }

        temp_x_ = min_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = min_x;
        temp_y_ = max_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = max_x;
        temp_y_ = max_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = max_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);

        temp_x_ = min_x;
        temp_y_ = min_y;
        p.x     = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
        p.y     = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
        p.z     = 0;
        points.points.push_back(p);
        maker_temp.markers.push_back(points);
      }
      max_index = max(max_index, id_index);
      if (id_index < max_index)
      {
        for (size_t i = (id_index + 1); i < max_index; i++)
        {
          visualization_msgs::Marker points;

          points.header.frame_id    = "/odom";
          points.header.stamp       = ros::Time::now();
          points.ns                 = "/per/lidar_obstacle_fusion";
          points.action             = visualization_msgs::Marker::DELETE;
          points.pose.orientation.w = 1.0;
          points.id                 = i;
          points.type               = visualization_msgs::Marker::LINE_STRIP;

          maker_temp.markers.push_back(points);
        }
      }
    }

    obstacle_cam_lidar_pub.publish(maker_temp);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "per_sender");
  ros::NodeHandle n;

  if (argc == 3)
  {
    mode_tip_1 = atoi(argv[1]); // 2 模拟驱动
    mode_tip_2 = atoi(argv[2]);
  }
  else
  {
    mode_tip_1 = 1; // 0 1 融合感知模拟 2 感知融合前
    mode_tip_2 = 0; // 0 使用presacan数据 1 使用p2数据  2使用固定假数据
  }

  ros::Subscriber obstacle_sub        = n.subscribe("/prescan/obstacle_location", 10, recvObstacleCallback);
  ros::Subscriber fusion_location_sub = n.subscribe("/localization/fusion_msg", 10, recvFusionLocationCallback);

  ros::Subscriber obu_location_sub = n.subscribe("/perception/obstacle_info", 10, recvFusionObuCallback);
  // ros::Subscriber obu_location_sub = n.subscribe("/perception/obstacle_info_test", 10, recvFusionObuCallback);

  ultr_sub = n.subscribe("/drivers/perception/ultrasonic_obstacle_info", 5, recvObuCallback);

  ros::Publisher obstacle_point_pub =
      n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_msg", 10, true);
  ros::Publisher obstacle_fusion_pub =
      n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_fusion_msg", 10, true);

  obstacle_cam_lidar_pub =
      n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_cam_lidar_msg", 10, true);

  agv_control_pub = n.advertise< control_msgs::ADControlAGV >("/map/stop_msg", 10, true);

  //
  ros::Publisher obstacle_pub = n.advertise< perception_msgs::FusionDataInfo >("/perception/obstacle_info", 10, true);

  //融合感知测试
  // ros::Publisher cam_pub =
  //     n.advertise< perception_sensor_msgs::ObjectList >("/drivers/perception/camera_obstacle_info", 10, true);

  // cam_sub   = n.subscribe("/drivers/perception/camera_obstacle_info", 5, recvObuCallback);
  // lidar_sub = n.subscribe("/perception/detection_lidar", 5, recvObuCallback);

  ros::Publisher cam_pub = n.advertise< perception_sensor_msgs::ObjectList >("/perception/detection_camera", 10, true);
  ros::Publisher lidar_pub = n.advertise< perception_sensor_msgs::ObjectList >("/perception/detection_lidar", 10, true);

  ros::Rate loop_rate(20);

  int count = 0;

  while (ros::ok())
  {
    if (mode_tip_1 == 2)
    {
      new_odometry_send_tip = 1;
    }
    else if (mode_tip_1 == 1)
    {
      new_odometry_send_tip = 1;
    }
    else
    {
    }

    if (new_odometry_send_tip == 1)
    {
      new_odometry_send_tip = 0;

      ros::Time current_time = ros::Time::now();
      perception_msgs::FusionDataInfo per_obstacle;

      per_obstacle.header.stamp = current_time;

      double obstacle_x = -3;
      double obstacle_y = 99.1;

      double distance = (obstacle_x - fusion_location_x) * (obstacle_x - fusion_location_x) +
                        (obstacle_y - fusion_location_y) * (obstacle_y - fusion_location_y);
      if (distance < -50 * 50)
      {

        per_obstacle.obstacle_num = 1;

        common_msgs::ObstacleInfo obstacle_;

        if (mode_tip_1 == 1)
        {
          obstacle_data.ID       = 101;
          obstacle_data.lat      = -3;
          obstacle_data.lon      = 99.1;
          obstacle_data.height   = 1;
          obstacle_data.velocity = 0;
          //  obstacle_data.angle_x     = -(msg->pose.pose.orientation.x - 90); // Y轴方向角度
          obstacle_data.angle_x  = 90; // gh坐标
          obstacle_data.volume_x = 1;  //长
          obstacle_data.volume_y = 1;  //宽
          obstacle_data.volume_z = 0;  //高
        }

        obstacle_.id       = obstacle_data.ID;
        obstacle_.velocity = obstacle_data.velocity;
        obstacle_.theta    = obstacle_data.angle_x;
        float test_angle_x = -(obstacle_data.angle_x - 90);

        if (abs(test_angle_x) > 360.0)
        {
          test_angle_x = fmod((test_angle_x * 180 / M_PI), 360.0);
        }
        float radian = test_angle_x * M_PI / 180;
        ROS_INFO("radian = %f  and angle = %f and change angle = %f", radian, obstacle_data.angle_x, test_angle_x);

        obstacle_.peak[0].x =
            obstacle_data.lat + obstacle_data.volume_x / 2 * cos(radian) - obstacle_data.volume_y / 2 * sin(radian);
        obstacle_.peak[0].y =
            obstacle_data.lon + obstacle_data.volume_x / 2 * sin(radian) + obstacle_data.volume_y / 2 * cos(radian);

        obstacle_.peak[1].x =
            obstacle_data.lat + obstacle_data.volume_x / 2 * cos(radian) + obstacle_data.volume_y / 2 * sin(radian);
        obstacle_.peak[1].y =
            obstacle_data.lon + obstacle_data.volume_x / 2 * sin(radian) - obstacle_data.volume_y / 2 * cos(radian);

        obstacle_.peak[2].x =
            obstacle_data.lat - obstacle_data.volume_x / 2 * cos(radian) + obstacle_data.volume_y / 2 * sin(radian);
        obstacle_.peak[2].y =
            obstacle_data.lon - obstacle_data.volume_x / 2 * sin(radian) - obstacle_data.volume_y / 2 * cos(radian);

        obstacle_.peak[3].x =
            obstacle_data.lat - obstacle_data.volume_x / 2 * cos(radian) - obstacle_data.volume_y / 2 * sin(radian);
        obstacle_.peak[3].y =
            obstacle_data.lon - obstacle_data.volume_x / 2 * sin(radian) + obstacle_data.volume_y / 2 * cos(radian);

        per_obstacle.obstacles.push_back(obstacle_);

        ROS_INFO("(%f,%f) volume = (%f,%f)", obstacle_data.lat, obstacle_data.lon, obstacle_data.volume_x,
                 obstacle_data.volume_y);
        ROS_INFO("(%f,%f) (%f,%f)", obstacle_.peak[1].x, obstacle_.peak[1].y, obstacle_.peak[0].x, obstacle_.peak[0].y);
        ROS_INFO("(%f,%f) (%f,%f)", obstacle_.peak[2].x, obstacle_.peak[2].y, obstacle_.peak[3].x, obstacle_.peak[3].y);
      }
      else
      {
        per_obstacle.obstacle_num = 0;
      }

      if (mode_tip_1 == 1)
      {
        obstacle_pub.publish(per_obstacle);
        // ROS_INFO("FusionDataInfo,%lf", distance);
      }
      else if (mode_tip_1 == 3)
      {
        obstacle_pub.publish(per_obstacle);
      }
      else if (mode_tip_1 == 2)
      {
        perception_sensor_msgs::ObjectList cam_obstacle_list;
        cam_obstacle_list.header.stamp = current_time;

        cam_obstacle_list.sensor_index = 0;
        cam_obstacle_list.obstacle_num = 4;

        common_msgs::DetectionInfo obj_info_temp_;
        double test_x  = 0;
        double test_y  = 0;
        double test_lx = 0;
        double test_ly = 0;
        test_x         = -3;
        test_y         = 99.1;
        test_lx        = 1;
        test_ly        = 1;
        pushDetectionInfo(test_x, test_y, test_lx, test_ly, obj_info_temp_);
        obj_info_temp_.id = 0;
        cam_obstacle_list.object_list.push_back(obj_info_temp_);
        test_x  = 5;
        test_y  = 74;
        test_lx = 1;
        test_ly = 1;
        pushDetectionInfo(test_x, test_y, test_lx, test_ly, obj_info_temp_);
        obj_info_temp_.id = 1;
        cam_obstacle_list.object_list.push_back(obj_info_temp_);

        cam_pub.publish(cam_obstacle_list);

        perception_sensor_msgs::ObjectList lidar_obstacle_list;
        lidar_obstacle_list.header.stamp = current_time;
        lidar_obstacle_list.sensor_index = 1;
        lidar_obstacle_list.obstacle_num = 4;
        test_x                           = 5;
        test_y                           = 74;
        test_lx                          = 1;
        test_ly                          = 1;
        pushDetectionInfo(test_x, test_y, test_lx, test_ly, obj_info_temp_);
        obj_info_temp_.id = 0;
        lidar_obstacle_list.object_list.push_back(obj_info_temp_);

        lidar_pub.publish(lidar_obstacle_list);
        ROS_INFO("SenDataInfo");
      }

      visualization_msgs::MarkerArray maker_obs;
      visualization_msgs::Marker points_;
      int obt_id = 1;
      pushAGVMarker(fusion_location_x, fusion_location_y, fusion_location_heading, obt_id, points_);
      maker_obs.markers.push_back(points_);
      obstacle_point_pub.publish(maker_obs);

      if (fusion_tip == 1)
      {
        obstacle_fusion_pub.publish(maker_obs_temp);
        fusion_tip = 1;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  ROS_INFO("shutting down!\n");
  ros::shutdown();

  return 0;
}

void recvFusionObuCallback(const perception_msgs::FusionDataInfo::ConstPtr &obu_msg)
{
  // ROS_INFO("reve: obu %u", obu_msg->obstacle_num);
  if (obu_msg->obstacle_num > 0)
  {
    maker_obs_temp.markers.clear();
    int id_index = 0;
    for (size_t i = 0; i < obu_msg->obstacle_num; i++)
    {
      id_index++;
      visualization_msgs::Marker points;

      points.header.frame_id    = "/odom";
      points.header.stamp       = ros::Time::now();
      points.ns                 = "/per/agv_obstacle_fusion";
      points.action             = visualization_msgs::Marker::ADD;
      points.pose.orientation.w = 1.0;
      points.id                 = id_index;
      points.type               = visualization_msgs::Marker::LINE_STRIP;
      // points.type               = visualization_msgs::Marker::POINTS;
      // POINTS markers use x and y scale for width/height respectively
      points.scale.x = 0.3;
      points.scale.y = 0.3;
      // Points are green
      points.color.r = 1;
      points.color.g = 0;
      points.color.b = 1;
      points.color.a = 1.0;

      geometry_msgs::Point p;

      p.x = obu_msg->obstacles.at(i).peak.at(0).x;
      p.y = obu_msg->obstacles.at(i).peak.at(0).y;
      p.z = 0;
      points.points.push_back(p);
      p.x = obu_msg->obstacles.at(i).peak.at(1).x;
      p.y = obu_msg->obstacles.at(i).peak.at(1).y;
      p.z = 0;
      points.points.push_back(p);
      p.x = obu_msg->obstacles.at(i).peak.at(2).x;
      p.y = obu_msg->obstacles.at(i).peak.at(2).y;
      p.z = 0;
      points.points.push_back(p);
      p.x = obu_msg->obstacles.at(i).peak.at(3).x;
      p.y = obu_msg->obstacles.at(i).peak.at(3).y;
      p.z = 0;
      points.points.push_back(p);
      p.x = obu_msg->obstacles.at(i).peak.at(0).x;
      p.y = obu_msg->obstacles.at(i).peak.at(0).y;
      p.z = 0;
      points.points.push_back(p);
      maker_obs_temp.markers.push_back(points);
    }
    fusion_tip            = 1;
    new_odometry_send_tip = 1;
    // obstacle_fusion_pub.publish(maker_obs_temp);
  }
}

void pushDetectionInfo(const double &in_loc_x, const double &in_loc_y, const double &in_length_x_,
                       const double &in_length_y_, common_msgs::DetectionInfo &obj_info_)
{
  obj_info_.id        = 0;
  obj_info_.obj_class = 0;

  double in_loc_x_ = in_loc_x - fusion_location_x;
  double in_loc_y_ = in_loc_y - fusion_location_y;

  ROS_INFO("(%lf, %lf) -> (%lf, %lf)", in_loc_x, in_loc_y, in_loc_x_, in_loc_y_);

  double x_max = in_loc_x_ + in_length_x_ / 2;
  double x_min = in_loc_x_ - in_length_x_ / 2;
  double y_max = in_loc_y_ + in_length_y_ / 2;
  double y_min = in_loc_y_ - in_length_y_ / 2;

  obj_info_.peek[0].x          = x_max;
  obj_info_.peek[0].y          = y_max;
  obj_info_.peek[1].x          = x_max;
  obj_info_.peek[1].y          = y_min;
  obj_info_.peek[2].x          = x_min;
  obj_info_.peek[2].y          = y_min;
  obj_info_.peek[3].x          = x_min;
  obj_info_.peek[3].y          = y_max;
  obj_info_.state[0]           = x_min;
  obj_info_.state[1]           = y_min;
  obj_info_.state[2]           = x_max;
  obj_info_.state[3]           = y_max;
  obj_info_.measurement_cov[0] = 0;
  obj_info_.measurement_cov[1] = 0;
  obj_info_.measurement_cov[2] = 0;
  obj_info_.measurement_cov[3] = 0;
  obj_info_.measurement_cov[4] = 0;
  obj_info_.measurement_cov[5] = 0;

  obj_info_.measurement_cov[6]  = 0;
  obj_info_.measurement_cov[7]  = 0;
  obj_info_.measurement_cov[8]  = 0;
  obj_info_.measurement_cov[9]  = 0;
  obj_info_.measurement_cov[10] = 0;
  obj_info_.measurement_cov[11] = 0;

  obj_info_.measurement_cov[12] = 0;
  obj_info_.measurement_cov[13] = 0;
  obj_info_.measurement_cov[14] = 0;
  obj_info_.measurement_cov[15] = 0;
  obj_info_.measurement_cov[16] = 0;
  obj_info_.measurement_cov[17] = 0;

  obj_info_.measurement_cov[18] = 0;
  obj_info_.measurement_cov[19] = 0;
  obj_info_.measurement_cov[20] = 0;
  obj_info_.measurement_cov[21] = 0;
  obj_info_.measurement_cov[22] = 0;
  obj_info_.measurement_cov[23] = 0;

  obj_info_.measurement_cov[24] = 0;
  obj_info_.measurement_cov[25] = 0;
  obj_info_.measurement_cov[26] = 0;
  obj_info_.measurement_cov[27] = 0;
  obj_info_.measurement_cov[28] = 0;
  obj_info_.measurement_cov[29] = 0;

  obj_info_.measurement_cov[30] = 0;
  obj_info_.measurement_cov[31] = 0;
  obj_info_.measurement_cov[32] = 0;
  obj_info_.measurement_cov[33] = 0;
  obj_info_.measurement_cov[34] = 0;
  obj_info_.measurement_cov[35] = 0;

  obj_info_.confidence = 1;
}

void pushStaticMarker(const double &in_loc_x_, const double &in_loc_y_, const double &in_length_x_,
                      const double &in_length_y_, const int id_, string ns_, visualization_msgs::Marker &points)
{
  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = ns_;
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = id_;
  points.type               = visualization_msgs::Marker::POINTS;
  points.scale.x            = 0.4;
  points.scale.y            = 0.4;
  // Points are green
  points.color.r = 1;
  points.color.g = 1;
  points.color.b = 0;
  points.color.a = 1.0;

  double x_max = in_loc_x_ + in_length_x_ / 2;
  double x_min = in_loc_x_ - in_length_x_ / 2;
  double y_max = in_loc_y_ + in_length_y_ / 2;
  double y_min = in_loc_y_ - in_length_y_ / 2;

  geometry_msgs::Point p;
  p.x = x_max;
  p.y = y_max;
  p.z = 0;
  points.points.push_back(p);
  p.x = x_min;
  p.y = y_max;
  p.z = 0;
  points.points.push_back(p);
  p.x = x_min;
  p.y = y_min;
  p.z = 0;
  points.points.push_back(p);
  p.x = x_max;
  p.y = y_min;
  p.z = 0;
  points.points.push_back(p);
  p.x = x_max;
  p.y = y_max;
  p.z = 0;
  points.points.push_back(p);
}

void pushAGVMarker(const double x_, const double y_, const double radian_, const int id_,
                   visualization_msgs::Marker &points)
{
  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = "/per/agv_obstacle";
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = id_;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  // Points are green
  points.color.r = 0;
  points.color.g = 0;
  points.color.b = 1;
  points.color.a = 1.0;

  double agv_w = 3.0;
  double agv_l = 16.0;

  geometry_msgs::Point p;

  p.x = x_ + agv_l / 2 * sin(radian_) - agv_w / 2 * cos(radian_);
  p.y = y_ + agv_l / 2 * cos(radian_) + agv_w / 2 * sin(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = x_ + agv_l / 2 * sin(radian_) + agv_w / 2 * cos(radian_);
  p.y = y_ + agv_l / 2 * cos(radian_) - agv_w / 2 * sin(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = x_ - agv_l / 2 * sin(radian_) + agv_w / 2 * cos(radian_);
  p.y = y_ - agv_l / 2 * cos(radian_) - agv_w / 2 * sin(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = x_ - agv_l / 2 * sin(radian_) - agv_w / 2 * cos(radian_);
  p.y = y_ - agv_l / 2 * cos(radian_) + agv_w / 2 * sin(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = x_ + agv_l / 2 * sin(radian_) - agv_w / 2 * cos(radian_);
  p.y = y_ + agv_l / 2 * cos(radian_) + agv_w / 2 * sin(radian_);
  p.z = 0;
  points.points.push_back(p);
}

void pushPrescanMarker(const float radian_, const int id_, visualization_msgs::Marker &points)
{
  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = "/per/per_obstacle";
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = id_;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  // Points are green
  points.color.r = 0;
  points.color.g = 0;
  points.color.b = 1;
  points.color.a = 1.0;

  geometry_msgs::Point p;

  p.x = obstacle_data.lat + obstacle_data.volume_x / 2 * cos(radian_) - obstacle_data.volume_y / 2 * sin(radian_);
  p.y = obstacle_data.lon + obstacle_data.volume_x / 2 * sin(radian_) + obstacle_data.volume_y / 2 * cos(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = obstacle_data.lat + obstacle_data.volume_x / 2 * cos(radian_) + obstacle_data.volume_y / 2 * sin(radian_);
  p.y = obstacle_data.lon + obstacle_data.volume_x / 2 * sin(radian_) - obstacle_data.volume_y / 2 * cos(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = obstacle_data.lat - obstacle_data.volume_x / 2 * cos(radian_) + obstacle_data.volume_y / 2 * sin(radian_);
  p.y = obstacle_data.lon - obstacle_data.volume_x / 2 * sin(radian_) - obstacle_data.volume_y / 2 * cos(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = obstacle_data.lat - obstacle_data.volume_x / 2 * cos(radian_) - obstacle_data.volume_y / 2 * sin(radian_);
  p.y = obstacle_data.lon - obstacle_data.volume_x / 2 * sin(radian_) + obstacle_data.volume_y / 2 * cos(radian_);
  p.z = 0;
  points.points.push_back(p);
  p.x = obstacle_data.lat + obstacle_data.volume_x / 2 * cos(radian_) - obstacle_data.volume_y / 2 * sin(radian_);
  p.y = obstacle_data.lon + obstacle_data.volume_x / 2 * sin(radian_) + obstacle_data.volume_y / 2 * cos(radian_);
  p.z = 0;
  points.points.push_back(p);
}
