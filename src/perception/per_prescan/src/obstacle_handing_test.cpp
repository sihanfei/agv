#include "common_msgs/DetectionInfo.h"
#include "control_msgs/ADControlAGV.h"
#include "control_msgs/AGVStatus.h"
#include "geometry_msgs/PointStamped.h"
#include "location_msgs/FusionDataInfo.h"
#include "location_sensor_msgs/FixedLidarInfo.h"
#include "map_msgs/REFPointArray.h"
#include "perception_msgs/FusionDataInfo.h"
#include "perception_sensor_msgs/ObjectList.h"
#include "plan_msgs/DecisionInfo.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <algorithm>
#include <sstream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "glog_helper.h"

using namespace std;

using std::__cxx11::string;

#define LOCA_MAX_SIZE 10 * 10

ros::Subscriber fusion_location_sub; //融合定位
ros::Subscriber fusion_obstacle_sub; //融合感知结果
ros::Subscriber cam_sub;             //相机感知
ros::Subscriber lidar_sub;           //激光感知
ros::Subscriber ref_line_sub;        //参考线
ros::Subscriber task_click_sub;      //点击点
ros::Subscriber lidar_obu_sub;       // obu
ros::Subscriber agv_status_sub;      // agv status

ros::Publisher route_decision_pub;
ros::Publisher agv_control_pub;
ros::Publisher agv_point_pub;
ros::Publisher obstacle_point_pub;
ros::Publisher obstacle_cam_lidar_pub;
ros::Publisher rviz_point_pub_;
ros::Publisher rviz_boundary_point_pub_;
ros::Publisher rviz_left_boundary_point_pub_;
ros::Publisher obstacle_camera_pub;

ros::Publisher rviz_new_boundary_pub_;

int location_index               = 0;
int location_vec_tip             = 0;
int max_index                    = 0;
double max_duration              = 0.0;
double min_ddd                   = 100000.0;
int obstacle_detection_judge_tip = 0;
int lidar_detection_recieve_tip  = 0;
int cam_detection_recieve_tip    = 0;
int detection_count              = 0;

plan_msgs::DecisionInfo decision_info;
control_msgs::AGVStatus agv_status_info;

struct ObsPos
{
  int id;
  geometry_msgs::Point relative_pos;
  geometry_msgs::Point map_pos;
  geometry_msgs::Point grid_pos;
};

struct ObstacleJudgeStr
{
  int id;                       //障碍物序号
  int obs_type;                 //障碍物类型
  int sport_type;               //运动类型 运动或静止
  int size;                     //栅格点数量
  std::vector< ObsPos > pos;    //障碍物点集位置
  int obs_ref_index;            //障碍物距离agv最近点在参考线地图上的序号
  int agv_ref_index;            //检测时agv在参考线地图上的序号
  int lane_att;                 //最近点所在道路属性
  ObsPos closest_point_pos;     //障碍物距离agv最近点
  double ds_to_agv;             //障碍物距离agv最近点在参考线地图上距离agv的道路距离
  double dt_to_agv;             //障碍物距离agv最近点在参考线地图上距离agv的时间距离
  double velocity;              //障碍物全局速度
  int is_front;                 //障碍物是否在agv的前方
  int is_left;                  //障碍物是否在agv的左侧
  int is_right;                 //障碍物是否在agv的右侧
  int is_behind;                //障碍物是否在agv的后方
  int detection_times;          //障碍物被检测到的次数
  ros::Time s_time;             //进入时间
  ros::Time e_time;             //离开时间
  double per_during_time_front; //预测障碍物在前方区域的存在时间
  double per_during_time_left;  //预测障碍物在左侧区域的存在时间
  double per_during_time_right; //预测障碍物在右侧区域的存在时间
  double per_during_time_end;   //预测障碍物在后方区域的存在时间
  int warning_type;             //障碍物警告级别
};

struct ObstacleDetectionTemp
{
  int ref_index;
  u_int32_t obs_id;
  double min_x;
  double min_y;
  double max_x;
  double max_y;
  geometry_msgs::Point corner_point[4];
  int point_duing_lane_tip[4];
  double obs_distance[4];
  int min_index[4];
  int judge_tip[4];

  bool operator==(struct ObstacleDetectionTemp b) const
  {
    return this->obs_id == b.obs_id;
  }
  bool operator>(struct ObstacleDetectionTemp b) const
  {
    return this->obs_id > b.obs_id;
  }
  bool operator<(struct ObstacleDetectionTemp b) const
  {
    return this->obs_id < b.obs_id;
  }
};

std::vector< ObstacleDetectionTemp > obstacle_detection_vec;
std::vector< ObstacleDetectionTemp > obstacle_detection_left_vec;
std::vector< ObstacleDetectionTemp > obstacle_detection_right_vec;
std::vector< ObstacleDetectionTemp > obstacle_detection_front_vec;
std::vector< ObstacleJudgeStr > obstacle_around_agv_vec;

struct LocationTemp
{
  ros::Time time;
  int index;
  double pos_x;
  double pos_y;
  double heading;
  double speed_x;
  double speed_y;
  double total_v;
  int ref_index;
  int ref_pos;

  bool operator==(struct LocationTemp b) const
  {
    return this->index == b.index;
  }

  bool operator>(struct LocationTemp b) const
  {
    return this->time.toSec() > b.time.toSec();
  }

  bool operator<(struct LocationTemp b) const
  {
    return this->time.toSec() < b.time.toSec();
  }
};

struct ObstacleTemp
{
  LocationTemp befor_location;
  LocationTemp after_location;
  ros::Time time;
  int index;
  double pos_x;
  double pos_y;
  double heading;
  double speed_x;
  double speed_y;
};

struct TaskPoint
{
  double x;
  double y;
  double heading;
  double max_speed;
  int laneID;
  int task_click_recv_tip;
};

std::vector< common_msgs::DetectionInfo > lidar_detection_obs_vec;
std::vector< common_msgs::DetectionInfo > camera_detection_obs_vec;
LocationTemp lidar_location_obs_time;
LocationTemp camera_location_obs_time;
std::vector< LocationTemp > location_vec;
LocationTemp cur_location;
TaskPoint task_click;
map_msgs::REFPointArray rount_ref_line;
map_msgs::REFPointArray rount_ref_line_find_temp;
int route_ref_line_updata_tip = 0; //初始化为0,收到新的为1,发送到车后为0
int obu_reciver_tip           = 0;
geometry_msgs::Point obu_agv_distance;
LocationTemp start_loc;
int enable_fixed_lidar  = 0;
int recv_agv_status_tip = 0;

int generateRefLine(int cur_index_, int end_point_index_);

double mathPointDistanceSquare(double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

//加速度
double getPositiveAccSpeed(double last_v_, double acc_, double time_, double target_v)
{
  // double ret_v = target_v;
  double ret_v = last_v_;
  if (last_v_ < target_v)
  {
    ret_v = last_v_ + acc_ * time_;
  }
  if (ret_v > target_v)
  {
    ret_v = target_v;
  }
  return ret_v;
}
//减速度
double getNegativeAccSpeed(double last_v_, double acc_, double time_, double target_v)
{
  // double ret_v = target_v;
  double ret_v = last_v_;
  if (last_v_ > target_v)
  {
    ret_v = last_v_ + acc_ * time_;
  }
  if (ret_v < target_v)
  {
    ret_v = target_v;
  }
  return ret_v;
}

void pushAGVMarker(const double x_, const double y_, double radian_)
{
  visualization_msgs::MarkerArray maker_obs;
  visualization_msgs::Marker points;

  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = "/per/agv_obstacle";
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = 101;
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

  radian_ = radian_ * M_PI / 180;

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
  maker_obs.markers.push_back(points);
  agv_point_pub.publish(maker_obs);
}

void rvizPointPub(double x_, double y_)
{
  geometry_msgs::PointStamped psmsg_gps;
  psmsg_gps.header.stamp    = ros::Time::now();
  psmsg_gps.header.frame_id = "odom";
  psmsg_gps.point.x         = x_;
  psmsg_gps.point.y         = y_;
  psmsg_gps.point.z         = 0;
  rviz_point_pub_.publish(psmsg_gps);
}

void rvizBoundaryPointPub(double x_, double y_)
{
  geometry_msgs::PointStamped psmsg_gps;
  psmsg_gps.header.stamp    = ros::Time::now();
  psmsg_gps.header.frame_id = "odom";
  psmsg_gps.point.x         = x_;
  psmsg_gps.point.y         = y_;
  psmsg_gps.point.z         = 0;
  rviz_boundary_point_pub_.publish(psmsg_gps);
}

void rvizLeftBoundaryPointPub(double x_, double y_)
{
  geometry_msgs::PointStamped psmsg_gps;
  psmsg_gps.header.stamp    = ros::Time::now();
  psmsg_gps.header.frame_id = "odom";
  psmsg_gps.point.x         = x_;
  psmsg_gps.point.y         = y_;
  psmsg_gps.point.z         = 0;
  rviz_left_boundary_point_pub_.publish(psmsg_gps);
}

void agvStatusCallback(const control_msgs::AGVStatus::ConstPtr &agv_status_msg)
{
  agv_status_info.ActualSpd = agv_status_msg->ActualSpd;
  recv_agv_status_tip       = 1;
}

int findAgvLocation(double x_, double y_, double heading_, int last_pos_index, int isFastJudge, int isHeadingJudge)
{
  double last_distance     = -1.0;
  double cur_distance      = -1.0;
  double next_distance     = -1.0;
  double min_delta_heading = 360.0;
  double delta_heading     = 0.0;
  double min_distance      = 100;
  int min_index            = 0;
  int begin_index          = 3;

  if (!rount_ref_line.REF_line_INFO.empty())
  {
    int count = static_cast< int >(rount_ref_line.REF_line_INFO.size());

    if (last_pos_index > 40 && last_pos_index < count)
    {
      begin_index = last_pos_index - 30;
    }

    for (int i = begin_index; i < count; i++)
    {
      if (i > 2)
      {
        last_distance = mathPointDistanceSquare(x_, y_, rount_ref_line.REF_line_INFO.at(i - 2).rx,
                                                rount_ref_line.REF_line_INFO.at(i - 2).ry);
        cur_distance = mathPointDistanceSquare(x_, y_, rount_ref_line.REF_line_INFO.at(i - 1).rx,
                                               rount_ref_line.REF_line_INFO.at(i - 1).ry);
        next_distance = mathPointDistanceSquare(x_, y_, rount_ref_line.REF_line_INFO.at(i).rx,
                                                rount_ref_line.REF_line_INFO.at(i).ry);
        delta_heading     = abs(rount_ref_line.REF_line_INFO.at(i - 1).rtheta - heading_);
        min_delta_heading = min(min_delta_heading, delta_heading);
        // ROS_INFO("Find ref point at %d (%lf,%lf) ref heading:%lf agv:%lf", i - 1,
        //          rount_ref_line.REF_line_INFO.at(i - 1).rx, rount_ref_line.REF_line_INFO.at(i - 1).ry,
        //          rount_ref_line.REF_line_INFO.at(i - 1).rtheta, heading_);
        if (isFastJudge == 1)
        {
          if (cur_distance < last_distance && cur_distance < next_distance && cur_distance < 1)
          {
            //比当前最小偏差不应大10度，且总偏差不应大于40度
            if (isHeadingJudge == 1)
            {
              if (delta_heading < (min_delta_heading + 10) && delta_heading < 40)
              {
                // ROS_INFO("Find agv point (%lf,%lf) the closest ref point at %d (%lf,%lf)", x_, y_, i - 1,
                //          rount_ref_line.REF_line_INFO.at(i - 1).rx, rount_ref_line.REF_line_INFO.at(i - 1).ry);
                // rvizPointPub(rount_ref_line.REF_line_INFO.at(i - 1).rx, rount_ref_line.REF_line_INFO.at(i - 1).ry);
                return (i - 1);
              }
            }
            else
            {
              return (i - 1);
            }
          }
          if (min_distance > cur_distance)
          {
            min_distance = cur_distance;
            min_index    = i - 1;
          }
        }
        else
        {
          if (isHeadingJudge == 1)
          {
            if (delta_heading > 80)
            {
              if (min_distance > cur_distance)
              {
                min_distance = cur_distance;
                min_index    = i - 1;
              }
            }
          }
          else
          {
            if (min_distance > cur_distance)
            {
              min_distance = cur_distance;
              min_index    = i - 1;
            }
          }
        }
      }
    }
    // ROS_INFO("Find all ref choice the min %d", min_index);
    return min_index;
  }
  else
  {
    return min_index;
  }
}

void updataAGVLocationCache()
{
}

void recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
  location_index++;
  cur_location.time  = location_msg->header.stamp;
  cur_location.index = location_index;
  cur_location.pos_x = location_msg->pose.x;
  cur_location.pos_y = location_msg->pose.y;

  // cur_location.pos_x = -190.029;
  // cur_location.pos_y = -45.2474;

  cur_location.heading = location_msg->yaw;
  cur_location.speed_x = location_msg->velocity.linear.x;
  cur_location.speed_y = location_msg->velocity.linear.y;
  cur_location.total_v = abs(cur_location.speed_x) + abs(cur_location.speed_y); // 1m/s
  //更新缓冲区
  if (!location_vec.empty())
  {
    if (cur_location.time.toSec() < location_vec.rbegin()->time.toSec())
    {
      location_vec.clear();
      location_vec_tip = 0;
    }
  }

  location_vec.push_back(cur_location);
  if (location_vec.size() > LOCA_MAX_SIZE)
  {
    location_vec_tip                        = 1;
    std::vector< LocationTemp >::iterator k = location_vec.begin();
    location_vec.erase(k); //删除第一个元素
  }
  cur_location.ref_index =
      findAgvLocation(cur_location.pos_x, cur_location.pos_y, cur_location.heading, cur_location.ref_index, 0, 0);
}

void recvFusionObuCallback(const perception_msgs::FusionDataInfo::ConstPtr &obu_msg)
{
}

void recvCamObsCallback(const perception_sensor_msgs::ObjectList::ConstPtr &msg)
{
  int obs_num = ( int )(msg->obstacle_num);
  if (obs_num > 0)
  {
    // ros::Time lidar_in = ros::Time::now();
    ROS_INFO("recv cam obs %d", obs_num);
    camera_detection_obs_vec.clear();
    camera_location_obs_time.time = msg->header.stamp;
    camera_detection_obs_vec      = msg->object_list;
  }
  cam_detection_recieve_tip = 1;
}

void recvClickCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  task_click.x                   = msg->point.x;
  task_click.y                   = msg->point.y;
  task_click.laneID              = ( int )msg->point.z;
  task_click.task_click_recv_tip = 1;
  ROS_INFO("rcev rviz click: (%lf,%lf) in lane %d", task_click.x, task_click.y, task_click.laneID);
}

// 1、车辆在参考线上定位 OK
// 2、选择车行前方向60m数据 OK
// 3、遍历障碍物4个边缘点距离车道线中心距离，当最小距离小于当前车道宽度时，计算障碍物与车道重合范围，判断4钟重合方式（是否满足换道条件）
// OK
// 4、计算所有障碍物占用情况，判断是换道、跟车、还是停车。 doing
// 5、下发执行命令。
// 障碍物在车前行方向60m，遍历障碍物位置，计算4个边缘点，计算障碍物距离道路边缘位置；
int getLocationFromVectorTemp(LocationTemp &cur_lidar_temp)
{
  //获取lidar时间对应的location
  LocationTemp befor_lidar_temp;
  LocationTemp after_lidar_temp;

  befor_lidar_temp.index = 0;
  after_lidar_temp.index = 0;

  int before_tip = 0;
  int after_tip  = 0;

  int count = static_cast< int >(location_vec.size());
  if (count > 10 && !location_vec.empty())
  {
    // ROS_INFO("cur Time %.8lf,vec-b: %.8lf,vec-e: %.8lf duration time: %.8lf (s)",
    //          cur_lidar_temp.time.toSec(), location_vec.begin()->time.toSec(), location_vec.rbegin()->time.toSec(),
    //          location_vec.rbegin()->time.toSec() - location_vec.begin()->time.toSec());

    //使用bag时出错
    if (location_vec.rbegin()->time.toSec() - cur_lidar_temp.time.toSec() < 0)
    {
      ROS_WARN("cur Time %.8lf,cur->b: %.8lf (s), cur->e: %.8lf (s)", cur_lidar_temp.time.toSec(),
               cur_lidar_temp.time.toSec() - location_vec.begin()->time.toSec(),
               location_vec.rbegin()->time.toSec() - cur_lidar_temp.time.toSec());
      return -1;
    }

    for (int i = 1; i < count; i++)
    {
      befor_lidar_temp = location_vec.at(i);
      // if (cur_lidar_temp.time.toSec() > location_vec.at(i).time.toSec())
      if (cur_lidar_temp < location_vec.at(i))
      {
        befor_lidar_temp = location_vec.at(i - 1);
        before_tip       = 1;
        // ROS_INFO("before time %d duration: %.8lf (s)", i - 1,
        //          cur_lidar_temp.time.toSec() - befor_lidar_temp.time.toSec());
        break;
      }
    }

    // for (int i = count - 2; i > 0; i--)
    for (int i = count - 1; i > 0; i--)
    {
      // if (i > (count - 1))
      // {
      //   break;
      // }

      after_lidar_temp = location_vec.at(i);
      // if (cur_lidar_temp.time.toSec() < location_vec.at(i).time.toSec())
      if (cur_lidar_temp > location_vec.at(i))
      {
        after_lidar_temp = location_vec.at(i + 1);
        after_tip        = 1;
        // ROS_INFO("after  time %d duration: %.8lf (s)", i + 1,
        //          after_lidar_temp.time.toSec() - cur_lidar_temp.time.toSec());
        break;
      }
    }

    if (after_lidar_temp.index > befor_lidar_temp.index && befor_lidar_temp.index > 0 && after_tip == 1 &&
        before_tip == 1)
    {
      double time_coefficient = (cur_lidar_temp.time.toSec() - befor_lidar_temp.time.toSec()) /
                                (after_lidar_temp.time.toSec() - befor_lidar_temp.time.toSec());
      // if (time_coefficient > 1)
      // {
      //   ROS_ERROR("time_coefficient :%lf", time_coefficient);
      // }

      cur_lidar_temp.pos_x =
          befor_lidar_temp.pos_x + (after_lidar_temp.pos_x - befor_lidar_temp.pos_x) * time_coefficient;
      cur_lidar_temp.pos_y =
          befor_lidar_temp.pos_y + (after_lidar_temp.pos_y - befor_lidar_temp.pos_y) * time_coefficient;
      cur_lidar_temp.speed_x =
          befor_lidar_temp.speed_x + (after_lidar_temp.speed_x - befor_lidar_temp.speed_x) * time_coefficient;
      cur_lidar_temp.speed_y =
          befor_lidar_temp.speed_y + (after_lidar_temp.speed_y - befor_lidar_temp.speed_y) * time_coefficient;
      if (abs(after_lidar_temp.heading - befor_lidar_temp.heading) > 180)
      {
        after_lidar_temp.heading = after_lidar_temp.heading + 360.0;
      }
      cur_lidar_temp.heading =
          befor_lidar_temp.heading + (after_lidar_temp.heading - befor_lidar_temp.heading) * time_coefficient;
      if (cur_lidar_temp.heading >= 360.0)
      {
        cur_lidar_temp.heading = cur_lidar_temp.heading - 360.0;
      }
      // ROS_INFO("before (%lf %lf):%lf  after (%lf %lf):%lf  time_coefficient :%lf", after_lidar_temp.pos_x,
      //          after_lidar_temp.pos_y, after_lidar_temp.heading, befor_lidar_temp.pos_x, befor_lidar_temp.pos_y,
      //          befor_lidar_temp.heading, time_coefficient);

      return 1;
    }
    else
    {
      ROS_WARN("before Time %d %.8lf -- after Time %d %.8lf", befor_lidar_temp.index,
               cur_lidar_temp.time.toSec() - befor_lidar_temp.time.toSec(), after_lidar_temp.index,
               after_lidar_temp.time.toSec() - cur_lidar_temp.time.toSec());
      return -1;
    }
  }
  else
  {
    return -1;
  }
}

void lidarRvizPusInit(visualization_msgs::Marker &points, int id_index, int color_r, int color_g, int color_b)
{
  points.header.frame_id = "/odom";
  points.header.stamp    = ros::Time::now();
  points.ns              = "/per/new_lidar_obstacle_show";
  // points.action             = visualization_msgs::Marker::ADD;
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = id_index;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  // Points are green
  points.color.r = color_r;
  points.color.g = color_g;
  points.color.b = color_b;
  points.color.a = 1.0;
}

void camRvizPusInit(visualization_msgs::Marker &points, int id_index, int color_r, int color_g, int color_b)
{
  points.header.frame_id = "/odom";
  points.header.stamp    = ros::Time::now();
  points.ns              = "/per/new_cam_obstacle_show";
  // points.action             = visualization_msgs::Marker::ADD;
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = id_index;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  // Points are green
  points.color.r = color_r;
  points.color.g = color_g;
  points.color.b = color_b;
  points.color.a = 1.0;
}

void camRvizPusClear(visualization_msgs::Marker &points, int id_index)
{
  points.header.frame_id = "/odom";
  points.header.stamp    = ros::Time::now();
  points.ns              = "/per/new_cam_obstacle_show";
  // points.action             = visualization_msgs::Marker::DELETE;
  points.action             = visualization_msgs::Marker::DELETEALL;
  points.pose.orientation.w = 1.0;
  points.id                 = id_index;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
}

void lidarRvizPusClear(visualization_msgs::Marker &points, int id_index)
{
  points.header.frame_id = "/odom";
  points.header.stamp    = ros::Time::now();
  points.ns              = "/per/new_lidar_obstacle_show";
  // points.action             = visualization_msgs::Marker::DELETE;
  points.action             = visualization_msgs::Marker::DELETEALL;
  points.pose.orientation.w = 1.0;
  points.id                 = id_index;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
}

void doAgvTransformationToLocal(double temp_x_, double temp_y_, double x_, double y_, double radian_,
                                geometry_msgs::Point &p)
{
  p.x = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
  p.y = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
  p.z = 0;
}

int isPointDuingCurLane(int min_index, geometry_msgs::Point &p, double min_distance)
{
  //按照当前航线当前车道宽度
  // ref位置的车道宽度 //斜行全车道 //
}

int isPointDuingLane(int min_index, geometry_msgs::Point &p, double min_distance, int &boundary_id_, int isFindAll)
{
  // ROS_INFO("isPointDuingLane");
  double amp_temp = 1.2;
  double min_dis  = 100;
  //计算障碍物顶点是否在车道上
  int count = static_cast< int >(rount_ref_line.REF_line_INFO.at(min_index).lane_ranges.size());
  for (int i = 0; i < count; i++)
  {
    geometry_msgs::Point ref_point;
    geometry_msgs::Point left_point;
    geometry_msgs::Point right_point;

    ref_point.x = rount_ref_line.REF_line_INFO.at(min_index).rx;
    ref_point.y = rount_ref_line.REF_line_INFO.at(min_index).ry;

    double left_boundary_  = rount_ref_line.REF_line_INFO.at(min_index).lane_ranges.at(i).left_boundary;
    double right_boundary_ = rount_ref_line.REF_line_INFO.at(min_index).lane_ranges.at(i).right_boundary;
    double radian_         = (rount_ref_line.REF_line_INFO.at(min_index).rtheta) * M_PI / 180;

    left_point.x  = ref_point.x + left_boundary_ * cos(radian_);
    left_point.y  = ref_point.y - left_boundary_ * sin(radian_);
    right_point.x = ref_point.x + right_boundary_ * cos(radian_);
    right_point.y = ref_point.y - right_boundary_ * sin(radian_);

    double left_distance  = mathPointDistanceSquare(p.x, p.y, left_point.x, left_point.y);
    double right_distance = mathPointDistanceSquare(p.x, p.y, right_point.x, right_point.y);

    double b_dis =
        mathPointDistanceSquare(p.x, p.y, (left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2);

    if (min_dis > b_dis)
    {
      min_dis      = b_dis;
      boundary_id_ = i + 1;
    }

    // min_ddd = min(min_ddd, (min_distance + left_distance + right_distance));

    // ROS_INFO("obs distance %lf -- boundary: %lf -- min_dd: %lf", (min_distance + left_distance + right_distance),
    //          (left_boundary_ * left_boundary_ + right_boundary_ * right_boundary_) * amp_temp, min_ddd);
    if (isFindAll == 0)
    {
      if ((left_boundary_ * left_boundary_ + right_boundary_ * right_boundary_) * amp_temp -
                  (left_distance + right_distance) >
              0 &&
          (rount_ref_line.REF_line_INFO.at(min_index).rs - rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs) <
              40.0)
      {
        // ROS_INFO("obs distance %lf -- boundary: %lf", (left_distance + right_distance),
        //          (left_boundary_ * left_boundary_ + right_boundary_ * right_boundary_) * amp_temp);
        // ROS_INFO("obs (%lf,%lf) ----->>>>> on ref line --%lf--", p.x, p.y,
        //          rount_ref_line.REF_line_INFO.at(min_index).rs -
        //              rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs);

        return 1;
      }
    }

    // if (rount_ref_line.REF_line_INFO.at(min_index).rs - rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs <
    //         40.0 &&
    //     rount_ref_line.REF_line_INFO.at(min_index).rs - rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs >
    //         0.0)
    // {
    //   ROS_INFO("obs (%lf,%lf) ----- %lf >>>>> on ref line", p.x, p.y,
    //            rount_ref_line.REF_line_INFO.at(min_index).rs -
    //                rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs);
    //   return 1;
    // }
  }
  if (isFindAll == 1)
  {
    return boundary_id_;
  }
  else
  {
    return 0;
  }
}

int isObsduringRef(ObstacleDetectionTemp &obs_temp)
{

  int ref_index_          = cur_location.ref_index;
  int count_route_ref     = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  int count_path_data_ref = static_cast< int >(decision_info.path_data_REF.size());

  double cur_rs = rount_ref_line.REF_line_INFO.at(ref_index_).rs;
  double min_distance[4];
  double ref_distance_tip = 40.0;
  for (int j = 0; j < 4; j++)
  {
    obs_temp.min_index[j] = -1;
    min_distance[j]       = 80 * 80;
    obs_temp.judge_tip[j] = 0;
  }

  for (int i = ref_index_; i < count_path_data_ref; i++)
  {
  }

  for (int i = ref_index_; i < count_route_ref; i++)
  {
    double &ref_x = rount_ref_line.REF_line_INFO.at(i).rx;
    double &ref_y = rount_ref_line.REF_line_INFO.at(i).ry;
    if ((rount_ref_line.REF_line_INFO.at(i).rs - cur_rs) > ref_distance_tip)
    {
      break;
    }
    for (int j = 0; j < 4; j++)
    {
      geometry_msgs::Point &corner_point = obs_temp.corner_point[j];
      obs_temp.obs_distance[j]           = mathPointDistanceSquare(corner_point.x, corner_point.y, ref_x, ref_y);
      if (min_distance[j] > obs_temp.obs_distance[j])
      {
        min_distance[j]       = obs_temp.obs_distance[j];
        obs_temp.min_index[j] = i;
      }
    }
  }
  int judge_tip_ = 0;
  for (int j = 0; j < 4; j++)
  {
    if (obs_temp.min_index[j] != -1)
    {
      int boundary_id = 0;

      // obs_temp.point_duing_lane_tip[j] = isPointDuingCurLane();

      // obs_temp.point_duing_lane_tip[j] =
      //     isPointDuingLane(obs_temp.min_index[j], obs_temp.corner_point[j], obs_temp.obs_distance[j], boundary_id,
      //     0);
      obs_temp.judge_tip[j] = obs_temp.point_duing_lane_tip[j];
      judge_tip_ += obs_temp.judge_tip[j];
    }
  }

  if (judge_tip_ > 0)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

int findMinDistaceObsAndRef(ObstacleDetectionTemp &obs_temp)
{
  // ROS_WARN("ref_index %d error", obs_temp.ref_index);
  // int ref_index_ = obs_temp.ref_index;
  int ref_index_ = cur_location.ref_index;
  int count      = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  // if (ref_index_ >= count - 2)
  // {
  //   ROS_WARN("ref_index %d error", ref_index_);
  //   return -1;
  // }
  double cur_rs = rount_ref_line.REF_line_INFO.at(ref_index_).rs;
  double min_distance[4];
  double ref_distance_tip = 60.0;

  for (int j = 0; j < 4; j++)
  {
    obs_temp.min_index[j] = -1;
    min_distance[j]       = 80 * 80;
    obs_temp.judge_tip[j] = 0;
  }

  //判断是否在前方车道内
  for (int i = ref_index_; i < (count - 2); i++)
  // for (int i = 0; i < count; i++)
  {
    double &ref_x = rount_ref_line.REF_line_INFO.at(i).rx;
    double &ref_y = rount_ref_line.REF_line_INFO.at(i).ry;

    if ((rount_ref_line.REF_line_INFO.at(i).rs - cur_rs) > ref_distance_tip)
    {
      break;
    }

    for (int j = 0; j < 4; j++)
    {
      geometry_msgs::Point &corner_point = obs_temp.corner_point[j];
      obs_temp.obs_distance[j]           = mathPointDistanceSquare(corner_point.x, corner_point.y, ref_x, ref_y);
      if (min_distance[j] > obs_temp.obs_distance[j])
      {
        min_distance[j]       = obs_temp.obs_distance[j];
        obs_temp.min_index[j] = i;
      }
    }
  }
  // ROS_WARN("min_distance: %d %lf  %d %lf  %d %lf  %d %lf", min_index1, min_distance1, min_index2, min_distance2,
  //          min_index3, min_distance3, min_index4, min_distance4);
  int judge_tip_ = 0;
  for (int j = 0; j < 4; j++)
  {
    if (obs_temp.min_index[j] != -1)
    {
      int boundary_id = 0;
      obs_temp.point_duing_lane_tip[j] =
          isPointDuingLane(obs_temp.min_index[j], obs_temp.corner_point[j], obs_temp.obs_distance[j], boundary_id, 0);
      obs_temp.judge_tip[j] = obs_temp.point_duing_lane_tip[j];
      judge_tip_ += obs_temp.judge_tip[j];
    }
  }

  if (judge_tip_ > 0)
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

void showRefBoundary(int ref_index, int lane_index)
{
  common_msgs::REFPoint ref_point_temp = rount_ref_line.REF_line_INFO.at(ref_index);

  int count = ref_point_temp.lane_ranges.empty() ? -1 : static_cast< int >(ref_point_temp.lane_ranges.size());
  if (count > 0)
  {
    if (lane_index > 0)
    {
      double temp_y_       = 0;
      double temp_left_x_  = 0 - ref_point_temp.lane_ranges.at(lane_index - 1).left_boundary;
      double temp_right_x_ = 0 - ref_point_temp.lane_ranges.at(lane_index - 1).right_boundary;
      double radian_       = (ref_point_temp.rtheta) * M_PI / 180;
      double left_x        = ref_point_temp.rx - temp_left_x_ * cos(radian_);
      double left_y        = ref_point_temp.ry + temp_left_x_ * sin(radian_);
      double right_x       = ref_point_temp.rx - temp_right_x_ * cos(radian_);
      double right_y       = ref_point_temp.ry + temp_right_x_ * sin(radian_);

      //    rvizPointPub(ref_point_temp.rx, ref_point_temp.ry);
      rvizBoundaryPointPub(right_x, right_y);
      rvizLeftBoundaryPointPub(left_x, left_y);
    }
    else
    {
      for (int i = 0; i < count; i++)
      {
        double temp_y_       = 0;
        double temp_left_x_  = 0 - ref_point_temp.lane_ranges.at(i).left_boundary;
        double temp_right_x_ = 0 - ref_point_temp.lane_ranges.at(i).right_boundary;
        double radian_       = (ref_point_temp.rtheta) * M_PI / 180;
        double left_x        = ref_point_temp.rx - temp_left_x_ * cos(radian_);
        double left_y        = ref_point_temp.ry + temp_left_x_ * sin(radian_);
        double right_x       = ref_point_temp.rx - temp_right_x_ * cos(radian_);
        double right_y       = ref_point_temp.ry + temp_right_x_ * sin(radian_);

        //    rvizPointPub(ref_point_temp.rx, ref_point_temp.ry);
        rvizBoundaryPointPub(right_x, right_y);
        rvizLeftBoundaryPointPub(left_x, left_y);
        // sleep(1);
        // ROS_INFO("showRefBoundary %d point (%lf,%lf) left %lf  right %lf", ref_index, ref_point_temp.rx,
        // ref_point_temp.ry,
        //          ref_point_temp.lane_ranges.at(i).left_boundary, ref_point_temp.lane_ranges.at(i).right_boundary);
      }
    }
  }
  else
  {
    ROS_WARN("This point (%lf,%lf) dont have boundary", ref_point_temp.rx, ref_point_temp.ry);
  }
}

void detectionLidarShowTest()
{
  visualization_msgs::MarkerArray maker_temp;
  visualization_msgs::Marker points;

  //清除rviz显示
  lidarRvizPusClear(points, 0);
  maker_temp.markers.push_back(points);
  obstacle_cam_lidar_pub.publish(maker_temp);
  maker_temp.markers.clear();

  //障碍物判断初值
  int id_index   = 0;
  double radian_ = cur_location.heading * M_PI / 180;
  double x_      = cur_location.pos_x;
  double y_      = cur_location.pos_y;
  double temp_x_ = 0;
  double temp_y_ = 0;
  //更新查询参考线位置

  int obs_num = static_cast< int >(lidar_detection_obs_vec.size());
  std::vector< common_msgs::DetectionInfo >::iterator lidar_it;
  for (lidar_it = lidar_detection_obs_vec.begin(); lidar_it != lidar_detection_obs_vec.end(); lidar_it++)
  {
    ObstacleDetectionTemp obstacle_temp;

    obstacle_temp.ref_index = 0;
    lidar_it->state[0];
    obstacle_temp.obs_id = lidar_it->id;
    obstacle_temp.max_x  = 0 - lidar_it->state[0];
    obstacle_temp.min_y  = lidar_it->state[1];
    obstacle_temp.min_x  = 0 - lidar_it->state[2];
    obstacle_temp.max_y  = lidar_it->state[3];

    double judge_y_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
    double judge_x_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
    if (judge_y_temp < 60.0)
    {
      // showRefBoundary( obstacle_temp.ref_index,0);
      //显示当前点在参考线路径边界上的位置
      id_index++;
      visualization_msgs::Marker points;
      //转换全局
      temp_x_ = obstacle_temp.min_x;
      temp_y_ = obstacle_temp.min_y;
      doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[0]);

      temp_x_ = obstacle_temp.min_x;
      temp_y_ = obstacle_temp.max_y;
      doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[1]);

      temp_x_ = obstacle_temp.max_x;
      temp_y_ = obstacle_temp.max_y;
      doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[2]);

      temp_x_ = obstacle_temp.max_x;
      temp_y_ = obstacle_temp.min_y;
      doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[3]);

      int judge_tip = 0;

      //显示和赋值
      if (judge_tip > 0)
      {
        // ROS_WARN("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
        //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
        lidarRvizPusInit(points, id_index, 1, 0, 0);
        points.points.push_back(obstacle_temp.corner_point[0]);
        points.points.push_back(obstacle_temp.corner_point[1]);
        points.points.push_back(obstacle_temp.corner_point[2]);
        points.points.push_back(obstacle_temp.corner_point[3]);
        points.points.push_back(obstacle_temp.corner_point[0]);
        maker_temp.markers.push_back(points);
      }
      else
      {
        // ROS_INFO("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
        //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
        lidarRvizPusInit(points, id_index, 0, 0, 0);
        points.points.push_back(obstacle_temp.corner_point[0]);
        points.points.push_back(obstacle_temp.corner_point[1]);
        points.points.push_back(obstacle_temp.corner_point[2]);
        points.points.push_back(obstacle_temp.corner_point[3]);
        points.points.push_back(obstacle_temp.corner_point[0]);
        maker_temp.markers.push_back(points);
      }
    }
  }
  obstacle_cam_lidar_pub.publish(maker_temp);
}

void recvLidarObsCallback(const perception_sensor_msgs::ObjectList::ConstPtr &msg)
{
  int obs_num = ( int )(msg->obstacle_num);
  if (obs_num > 0)
  {
    // ros::Time lidar_in = ros::Time::now();
    // ROS_INFO("recv lidar obs %d", obs_num);

    lidar_detection_obs_vec.clear();
    lidar_location_obs_time.time = msg->header.stamp;
    lidar_detection_obs_vec      = msg->object_list;
    ROS_INFO("recv lidar obs %d %lf", obs_num, lidar_location_obs_time.time.toSec());

    detectionLidarShowTest();
  }
  lidar_detection_recieve_tip = 0;
}

void recvRefCallback(const map_msgs::REFPointArray::ConstPtr &msg)
{
  rount_ref_line.header.stamp = msg->header.stamp;

  rount_ref_line.target_lane_ID = msg->target_lane_ID;
  rount_ref_line.agv_lane_ID    = msg->agv_lane_ID;

  rount_ref_line.REF_line_INFO.clear();

  int count_line = static_cast< int >(msg->REF_line_INFO.size());
  for (int i = 0; i < count_line; i++)
  {
    common_msgs::REFPoint ref_pinfo_temp;
    ref_pinfo_temp.rs             = msg->REF_line_INFO.at(i).rs;
    ref_pinfo_temp.rx             = msg->REF_line_INFO.at(i).rx;
    ref_pinfo_temp.ry             = msg->REF_line_INFO.at(i).ry;
    ref_pinfo_temp.rtheta         = msg->REF_line_INFO.at(i).rtheta;
    ref_pinfo_temp.max_speed      = msg->REF_line_INFO.at(i).max_speed;
    ref_pinfo_temp.rkappa         = msg->REF_line_INFO.at(i).rkappa;
    ref_pinfo_temp.rdkappa        = msg->REF_line_INFO.at(i).rdkappa;
    ref_pinfo_temp.max_speed      = msg->REF_line_INFO.at(i).max_speed;
    ref_pinfo_temp.lane_id        = msg->REF_line_INFO.at(i).lane_id;
    ref_pinfo_temp.line_direction = msg->REF_line_INFO.at(i).line_direction;

    int count_boundary = static_cast< int >(msg->REF_line_INFO.at(i).lane_ranges.size());
    for (int j = 0; j < count_boundary; j++)
    {
      common_msgs::LaneRange lane_range_;
      lane_range_.left_boundary  = msg->REF_line_INFO.at(i).lane_ranges.at(j).left_boundary;
      lane_range_.right_boundary = msg->REF_line_INFO.at(i).lane_ranges.at(j).right_boundary;
      ref_pinfo_temp.lane_ranges.push_back(lane_range_);
    }
    rount_ref_line.REF_line_INFO.push_back(ref_pinfo_temp);
  }
  route_ref_line_updata_tip = 1;

  ROS_ERROR("REF first point (%lf,%lf)", msg->REF_line_INFO.at(0).rx, msg->REF_line_INFO.at(0).ry);

  ROS_INFO("updata ref line lane %u -> %u total %d point %lf m", rount_ref_line.agv_lane_ID,
           rount_ref_line.target_lane_ID, static_cast< int >(rount_ref_line.REF_line_INFO.size()),
           rount_ref_line.REF_line_INFO.rbegin()->rs);

  generateRefLine(0, count_line - 1);
  route_ref_line_updata_tip = 0;
}

int getRefIndexFromRefline(int cur_ref_index_, double rs_tip_)
{
  int begin_ref_index_ = cur_ref_index_;
  int count            = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  for (int i = cur_location.ref_index; i < count; i++)
  {
    double delta_rs = rount_ref_line.REF_line_INFO.at(i).rs - rount_ref_line.REF_line_INFO.at(cur_ref_index_).rs;
    if (delta_rs > rs_tip_)
    {
      begin_ref_index_ = i;
      return begin_ref_index_;
    }
  }
  return -1;
}

//直行道路 直行减速
int generateLaneSlowDown(int begin_, int end_, common_msgs::PathPoint &last_ref_point_temp,
                         plan_msgs::DecisionInfo &decision_info)
{
  double negative_acc = -2.0; //负加速度
  double target_speed = 1.0;  // 1m/s
  int last_ref_index  = begin_;
  for (int i = begin_; i < end_; i++)
  {
    common_msgs::PathPoint new_ref_point_temp;
    new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(i).rx;
    new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(i).ry;
    new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;

    double distance_point = mathPointDistanceSquare(last_ref_point_temp.x, last_ref_point_temp.y, new_ref_point_temp.x,
                                                    new_ref_point_temp.y);
    new_ref_point_temp.v = last_ref_point_temp.v + negative_acc * (sqrt(distance_point) / last_ref_point_temp.v);
    if (new_ref_point_temp.v < target_speed)
    {
      new_ref_point_temp.v = target_speed;
    }
    decision_info.path_data_REF.push_back(new_ref_point_temp);
    // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  ---  acc", new_ref_point_temp.x, new_ref_point_temp.y,
    //          new_ref_point_temp.theta, new_ref_point_temp.v);
    last_ref_point_temp = new_ref_point_temp;
    last_ref_index      = i;

    if (last_ref_point_temp.v == target_speed)
    {
      // new_ref_point_temp.v = target_speed;
      break;
    }
  }
  return last_ref_index;
}

double getDiagonalDistanceY(double max_target_angle, double diagonal_distance_x_)
{
  double diagonal_distance_y_ = abs(diagonal_distance_x_) / tan(max_target_angle * M_PI / 180);
  return diagonal_distance_y_;
}

double getDiagonalDistanceX(int ref_index_, int lane_index_)
{
  double diagonal_distance_x_ = 0.0;
  if (lane_index_ + 1 >= static_cast< int >(rount_ref_line.REF_line_INFO.at(ref_index_).lane_ranges.size()))
  {
    return diagonal_distance_x_;
  }

  diagonal_distance_x_ = rount_ref_line.REF_line_INFO.at(ref_index_).lane_ranges.at(lane_index_).left_boundary +
                         rount_ref_line.REF_line_INFO.at(ref_index_).lane_ranges.at(lane_index_ + 1).right_boundary;
  diagonal_distance_x_ = 0 - diagonal_distance_x_;

  return diagonal_distance_x_;
}

int generateFirstCurvelLine(double diagonal_distance_x, double radian_, int begin_, int end_,
                            common_msgs::PathPoint &last_ref_point_temp, plan_msgs::DecisionInfo &decision_info)
{
  int last_ref_index  = begin_;
  double target_speed = 1.0; // 1m/s

  if ((begin_ + 1) < end_)
  {
    //第一个弯道平移
    for (int i = (begin_ + 1); i < end_; i++)
    {
      common_msgs::PathPoint new_ref_point_temp;
      new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(i).rx - diagonal_distance_x * cos(radian_);
      new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(i).ry + diagonal_distance_x * sin(radian_);
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
      new_ref_point_temp.v     = target_speed;
      if (abs(new_ref_point_temp.theta - last_ref_point_temp.theta) < 0.5)
      {
        //当前道路结束
        break;
      }
      decision_info.path_data_REF.push_back(new_ref_point_temp);
      // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  --  333", new_ref_point_temp.x, new_ref_point_temp.y,
      //          new_ref_point_temp.theta, new_ref_point_temp.v);
      last_ref_point_temp = new_ref_point_temp;
      last_ref_index      = i;
    }
  }

  return last_ref_index;
}

int generateAddlLine(int add_line_tip, double diagonal_distance_x, double radian_, int begin_, int end_,
                     common_msgs::PathPoint &last_ref_point_temp, plan_msgs::DecisionInfo &decision_info)
{
  int last_ref_index  = begin_;
  double target_speed = 1.0; // 1m/s

  if (add_line_tip > 0)
  {
    for (int j = 0; j < end_; j++)
    {
      common_msgs::PathPoint new_ref_point_temp;
      double diagonal_temp     = diagonal_distance_x - 0.5 * j;
      new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(last_ref_index + 1).rx - diagonal_temp * cos(radian_);
      new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(last_ref_index + 1).ry + diagonal_temp * sin(radian_);
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(last_ref_index + 1).rtheta;
      new_ref_point_temp.v     = target_speed;
      if (diagonal_temp < 0)
      {
        last_ref_point_temp = new_ref_point_temp;
        last_ref_index      = last_ref_index + 1;
        break;
      }
      decision_info.path_data_REF.push_back(new_ref_point_temp);
      // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  --  444", new_ref_point_temp.x, new_ref_point_temp.y,
      //          new_ref_point_temp.theta, new_ref_point_temp.v);
    }
  }
  else
  {
    for (int i = begin_; i < end_; i++)
    {
      common_msgs::PathPoint new_ref_point_temp;
      new_ref_point_temp.x = rount_ref_line.REF_line_INFO.at(i).rx;
      new_ref_point_temp.y = rount_ref_line.REF_line_INFO.at(i).ry;

      double distance_point = mathPointDistanceSquare(last_ref_point_temp.x, last_ref_point_temp.y,
                                                      new_ref_point_temp.x, new_ref_point_temp.y);
      //   if (distance_point > 4 * diagonal_distance_x * diagonal_distance_x)
      if (distance_point < 0.5 * 0.5)
      {
        last_ref_index = i;
        break;
      }
    }
  }

  return last_ref_index;
}

double generateEndSpeed(double last_v, double new_v, int index_, int begin_index_, int target_index_, int end_length,
                        double &end_begin_speed)
{
  double ret_v = new_v;
  //停车速度规划
  if (index_ == (target_index_ - end_length - 1))
  {
    end_begin_speed = last_v;
  }
  if (target_index_ - begin_index_ > (end_length))
  {
    //根据最后一次速度在三个点内停下来
    if (index_ >= (target_index_ - end_length))
    {
      ret_v = last_v - end_begin_speed / (end_length + 1);
      if (ret_v < 0)
      {
        ret_v = 0;
      }
    }
  }
  if (index_ >= target_index_)
  {
    ret_v = 0;
  }
  //停车速度规划
  return ret_v;
}

void generateSurpluslLine(int begin_, int end_, int taget_, common_msgs::PathPoint &last_ref_point_temp,
                          plan_msgs::DecisionInfo &decision_info)
{
  int last_ref_index      = begin_;
  double target_speed     = 1.0; // 1m/s
  double p_acc_           = 0.5;
  double n_acc_           = -2.0;
  int end_tip             = 0;
  double end_begin_speed  = 0;
  int end_point_judge_num = 3;

  for (int i = (begin_ + 1); i < end_; i++)
  {
    common_msgs::PathPoint new_ref_point_temp;
    new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(i).rx;
    new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(i).ry;
    new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
    double time_temp_ =
        (abs(new_ref_point_temp.y - last_ref_point_temp.y) + abs(new_ref_point_temp.x - last_ref_point_temp.x)) /
        last_ref_point_temp.v;

    if (rount_ref_line.REF_line_INFO.at(i).rkappa == 0 && rount_ref_line.REF_line_INFO.at(i).rdkappa == 0)
    {
      if (last_ref_point_temp.v < target_speed * 1.5)
      {
        new_ref_point_temp.v = getPositiveAccSpeed(last_ref_point_temp.v, p_acc_, time_temp_, target_speed * 1.5);
      }
      else
      {
        new_ref_point_temp.v = getNegativeAccSpeed(last_ref_point_temp.v, n_acc_, time_temp_, target_speed * 1.5);
      }
    }
    else
    {
      if (last_ref_point_temp.v < target_speed * 1.0)
      {
        new_ref_point_temp.v = getPositiveAccSpeed(last_ref_point_temp.v, p_acc_, time_temp_, target_speed * 1.0);
      }
      else
      {
        new_ref_point_temp.v = getNegativeAccSpeed(last_ref_point_temp.v, n_acc_, time_temp_, target_speed * 1.0);
      }
    }

    if (rount_ref_line.REF_line_INFO.at(i).line_direction == 8)
    {
      new_ref_point_temp.theta = last_ref_point_temp.theta;
      new_ref_point_temp.v     = target_speed * 0.5;
    }
    if (rount_ref_line.REF_line_INFO.at(i).line_direction == 16)
    {
      new_ref_point_temp.theta = last_ref_point_temp.theta;
      new_ref_point_temp.v     = target_speed * 0.5;
    }

    new_ref_point_temp.v = generateEndSpeed(last_ref_point_temp.v, new_ref_point_temp.v, i, begin_, taget_,
                                            end_point_judge_num, end_begin_speed);

    // //停车速度规划
    // if (i == (taget_ - end_point_judge_num - 1))
    // {
    //   end_tip         = 1;
    //   end_begin_speed = last_ref_point_temp.v;
    // }
    // if (taget_ - begin_ > (end_point_judge_num))
    // {
    //   //根据最后一次速度在三个点内停下来
    //   if (i >= (taget_ - end_point_judge_num))
    //   {
    //     new_ref_point_temp.v = last_ref_point_temp.v - end_begin_speed / (end_point_judge_num + 1);
    //     if (new_ref_point_temp.v < 0)
    //     {
    //       new_ref_point_temp.v = 0;
    //     }
    //   }
    // }
    // if (i >= taget_)
    // {
    //   new_ref_point_temp.v = 0;
    // }
    // //停车速度规划

    decision_info.path_data_REF.push_back(new_ref_point_temp);
    last_ref_point_temp = new_ref_point_temp;
    ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  -ID %d dir %d-  555 rkappa %lf rdkappa %lf", new_ref_point_temp.x,
             new_ref_point_temp.y, new_ref_point_temp.theta, new_ref_point_temp.v,
             rount_ref_line.REF_line_INFO.at(i).lane_id, rount_ref_line.REF_line_INFO.at(i).line_direction,
             rount_ref_line.REF_line_INFO.at(i).rkappa, rount_ref_line.REF_line_INFO.at(i).rdkappa);
  }
}

int generateDiagonalLine(double max_target_angle, double diagonal_distance_x, double diagonal_distance_y,
                         double radian_, int begin_, int end_, common_msgs::PathPoint &last_ref_point_temp,
                         plan_msgs::DecisionInfo &decision_info)
{
  int last_ref_index  = begin_;
  double target_speed = 1.0; // 1m/s

  for (int i = begin_; i < end_; i++)
  {
    common_msgs::PathPoint new_ref_point_temp;

    // double radian_ = rount_ref_line.REF_line_INFO.at(i).rtheta * M_PI / 180;
    double delta_rs = rount_ref_line.REF_line_INFO.at(i).rs - rount_ref_line.REF_line_INFO.at(begin_).rs;
    if (delta_rs > abs(diagonal_distance_y))
    {
      // radian_                  = rount_ref_line.REF_line_INFO.at(cur_ref_index).rtheta * M_PI / 180;
      new_ref_point_temp.v     = target_speed; //增加速度变换
      new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(i).rx - diagonal_distance_x * cos(radian_);
      new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(i).ry + diagonal_distance_x * sin(radian_);
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
      // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  --  222", new_ref_point_temp.x, new_ref_point_temp.y,
      //          new_ref_point_temp.theta, new_ref_point_temp.v);
    }
    else
    {
      new_ref_point_temp.v = target_speed;
      new_ref_point_temp.x =
          rount_ref_line.REF_line_INFO.at(i).rx - delta_rs * tan(max_target_angle * M_PI / 180) * cos(radian_);
      new_ref_point_temp.y =
          rount_ref_line.REF_line_INFO.at(i).ry + delta_rs * tan(max_target_angle * M_PI / 180) * sin(radian_);
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
      // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  --  111", new_ref_point_temp.x, new_ref_point_temp.y,
      //          new_ref_point_temp.theta, new_ref_point_temp.v);
    }

    if (abs(new_ref_point_temp.theta - last_ref_point_temp.theta) > 0.5)
    {
      //当前道路结束
      break;
    }
    decision_info.path_data_REF.push_back(new_ref_point_temp);
    last_ref_point_temp = new_ref_point_temp;
    last_ref_index      = i;
  }

  return last_ref_index;
}

int doAddLineJudge(double begin_angle, double end_angle, int change_right)
{
  double delta_angel = end_angle - begin_angle;
  int turn_right_tip = -1;

  if (delta_angel > 180)
  {
    delta_angel -= 360;
  }
  else if (delta_angel < -180)
  {
    delta_angel += 360;
  }
  else
  {
    delta_angel = delta_angel;
  }

  // 1 右转 0 左转
  if (delta_angel > 0)
  {
    turn_right_tip = 1;
  }

  //左斜，左转，减少长度
  if (change_right < 0 && turn_right_tip < 0)
  {
    return -1;
  }
  //右斜，右转，减少长度
  if (change_right > 0 && turn_right_tip > 0)
  {
    return -1;
  }
  //左斜，右转，增加长度
  if (change_right < 0 && turn_right_tip > 0)
  {
    return 1;
  }
  //右斜，左转，增加长度
  if (change_right > 0 && turn_right_tip < 0)
  {
    return 1;
  }
}

int generateNewRefLine(int change_right)
{
  //当前位置
  int cur_ref_index = cur_location.ref_index;
  int boundary_num  = rount_ref_line.REF_line_INFO.at(cur_ref_index).lane_ranges.size();
  //车道数量大于1可以执行切换

  if (boundary_num > 1)
  {
    // ros::Time lidar_out = ros::Time::now();
    // if (max_duration < (lidar_out.toSec() - lidar_in.toSec()))
    // {
    //   max_duration = lidar_out.toSec() - lidar_in.toSec();
    // }
    // ROS_ERROR("Lidar Judge use %0.8lf(s) MAX Dur is %0.8lf(s)", lidar_out.toSec() - lidar_in.toSec(), max_duration);
    ros::Time b_time = ros::Time::now();
    ROS_INFO("generate Diagonal Line");

    //车道大于可以Diagonal Line
    //获取起点位置3m
    // plan_msgs::DecisionInfo decision_info;

    decision_info.header.stamp    = ros::Time::now();
    decision_info.header.frame_id = "route_decision_info";
    decision_info.path_data_REF.clear();

    int old_ref_count          = static_cast< int >(rount_ref_line.REF_line_INFO.size());
    int turn_angle_begin_index = getRefIndexFromRefline(cur_ref_index, 3.0);
    if (turn_angle_begin_index < 0)
    {
      return -1;
    }

    //当前速度 在直线行驶中减速到1m/s
    double cur_speed = sqrt(cur_location.speed_x * cur_location.speed_x + cur_location.speed_y * cur_location.speed_y);

    common_msgs::PathPoint last_ref_point_temp;
    last_ref_point_temp.x     = cur_location.pos_x;
    last_ref_point_temp.y     = cur_location.pos_y;
    last_ref_point_temp.v     = cur_speed;
    last_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(cur_ref_index).rtheta;
    if (last_ref_point_temp.v < 0.5)
    {
      last_ref_point_temp.v = 0.5;
    }
    //对第一个点赋值
    decision_info.path_data_REF.push_back(last_ref_point_temp);
    ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  -ID %d dir %d-  000 rkappa %lf rdkappa %lf",
             last_ref_point_temp.x, last_ref_point_temp.y, last_ref_point_temp.theta, last_ref_point_temp.v,
             rount_ref_line.REF_line_INFO.at(cur_ref_index).lane_id,
             rount_ref_line.REF_line_INFO.at(cur_ref_index).line_direction,
             rount_ref_line.REF_line_INFO.at(cur_ref_index).rkappa,
             rount_ref_line.REF_line_INFO.at(cur_ref_index).rdkappa);

    int last_ref_index      = cur_ref_index;
    double radian_          = rount_ref_line.REF_line_INFO.at(cur_ref_index).rtheta * M_PI / 180;
    double target_speed     = 1.0; // 1m/s
    double max_target_angle = 30.0;
    int cur_lane_index = 0; //根据当前位置判断在第几个车道上，或者考虑按照宽度平移，待定
    double diagonal_distance_x = getDiagonalDistanceX(cur_ref_index, cur_lane_index);
    double diagonal_distance_y = getDiagonalDistanceY(max_target_angle, diagonal_distance_x);
    // double diagonal_distance   = diagonal_distance_x / sin(max_target_angle * M_PI / 180);
    if (change_right > 0)
    {
      max_target_angle = 0 - max_target_angle;
    }
    //直行道路 直行减速
    last_ref_index = generateLaneSlowDown(last_ref_index, turn_angle_begin_index, last_ref_point_temp, decision_info);
    //斜行 - 转弯     //计算斜行终点     //左- 右+
    last_ref_index = generateDiagonalLine(max_target_angle, diagonal_distance_x, diagonal_distance_y, radian_,
                                          last_ref_index, old_ref_count, last_ref_point_temp, decision_info);
    //第一个弯道平移
    double begin_angle = last_ref_point_temp.theta;
    last_ref_index     = generateFirstCurvelLine(diagonal_distance_x, radian_, last_ref_index, old_ref_count,
                                             last_ref_point_temp, decision_info);
    double end_angle = last_ref_point_temp.theta;

    //弯道后是否需要补充长度，或这减少长度  判断弯道前后角度变化，减少为左转，增加为右转
    int add_line_tip = doAddLineJudge(begin_angle, end_angle, change_right);

    last_ref_index = generateAddlLine(add_line_tip, diagonal_distance_x, radian_, last_ref_index, old_ref_count,
                                      last_ref_point_temp, decision_info);
    generateSurpluslLine(last_ref_index, old_ref_count, old_ref_count, last_ref_point_temp, decision_info);

    route_decision_pub.publish(decision_info);

    ros::Time e_time = ros::Time::now();
    ROS_INFO("generate Diagonal Line %d usetime %0.8lf(s) ,ole ref point %d", decision_info.path_data_REF.size(),
             e_time.toSec() - b_time.toSec(), old_ref_count - cur_ref_index);
    return 1;
  }
  else
  {
    //单车道无法生成Diagonal Line
    return -1;
  }
}

int generateParallelLine(double max_target_angle, double diagonal_distance_x, double max_ds, double radian_, int begin_,
                         int end_, int taget_, common_msgs::PathPoint &last_ref_point_temp,
                         plan_msgs::DecisionInfo &decision_info)
{

  int last_ref_index  = begin_;
  double target_speed = 1.0; // 1m/s

  for (int i = begin_; i < end_; i++)
  {
    common_msgs::PathPoint new_ref_point_temp;

    // double radian_ = rount_ref_line.REF_line_INFO.at(i).rtheta * M_PI / 180;
    double delta_rs = rount_ref_line.REF_line_INFO.at(i).rs - rount_ref_line.REF_line_INFO.at(begin_).rs;
    // if ((i - begin_) > 30)
    if (delta_rs > max_ds)
    {
      break;
    }
    else
    {
      new_ref_point_temp.v     = target_speed * 1.0; //增加速度变换
      new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(i).rx - diagonal_distance_x * cos(radian_);
      new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(i).ry + diagonal_distance_x * sin(radian_);
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
      ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  -ds %lf-  222", new_ref_point_temp.x, new_ref_point_temp.y,
               new_ref_point_temp.theta, new_ref_point_temp.v, delta_rs);
    }

    if (i > taget_)
    {
      new_ref_point_temp.v = 0;
    }

    decision_info.path_data_REF.push_back(new_ref_point_temp);
    last_ref_point_temp = new_ref_point_temp;
    last_ref_index      = i;
  }

  return last_ref_index;
}

int generateDiagonalLineBack(double max_target_angle, double diagonal_distance_x, double diagonal_distance_y,
                             double radian_, int begin_, int end_, int taget_,
                             common_msgs::PathPoint &last_ref_point_temp, plan_msgs::DecisionInfo &decision_info)
{
  int last_ref_index  = begin_;
  double target_speed = 1.0; // 1m/s

  for (int i = begin_; i < end_; i++)
  {
    common_msgs::PathPoint new_ref_point_temp;

    // double radian_ = rount_ref_line.REF_line_INFO.at(i).rtheta * M_PI / 180;
    double delta_rs = rount_ref_line.REF_line_INFO.at(i).rs - rount_ref_line.REF_line_INFO.at(begin_).rs;
    if (delta_rs > abs(diagonal_distance_y))
    {
      // radian_                  = rount_ref_line.REF_line_INFO.at(cur_ref_index).rtheta * M_PI / 180;
      new_ref_point_temp.v     = target_speed * 1.0; //增加速度变换
      new_ref_point_temp.x     = rount_ref_line.REF_line_INFO.at(i).rx;
      new_ref_point_temp.y     = rount_ref_line.REF_line_INFO.at(i).ry;
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
      // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  --  222", new_ref_point_temp.x, new_ref_point_temp.y,
      //          new_ref_point_temp.theta, new_ref_point_temp.v);
    }
    else
    {

      new_ref_point_temp.v = target_speed * 0.5;
      delta_rs             = 0 - delta_rs;
      new_ref_point_temp.x = rount_ref_line.REF_line_INFO.at(i).rx - diagonal_distance_x * cos(radian_) -
                             delta_rs * tan(max_target_angle * M_PI / 180) * cos(radian_);
      new_ref_point_temp.y = rount_ref_line.REF_line_INFO.at(i).ry + diagonal_distance_x * sin(radian_) +
                             delta_rs * tan(max_target_angle * M_PI / 180) * sin(radian_);
      new_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(i).rtheta;
      // ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  --  111", new_ref_point_temp.x, new_ref_point_temp.y,
      //          new_ref_point_temp.theta, new_ref_point_temp.v);
    }

    if (i > taget_)
    {
      new_ref_point_temp.v = 0;
    }

    if (abs(new_ref_point_temp.theta - last_ref_point_temp.theta) > 0.5)
    {
      //当前道路结束
      break;
    }

    decision_info.path_data_REF.push_back(new_ref_point_temp);
    last_ref_point_temp = new_ref_point_temp;
    last_ref_index      = i;
  }

  return last_ref_index;
}

int generateRefLine(int cur_index_, int end_point_index_)
{
  ros::Time b_time = ros::Time::now();
  ROS_INFO("generate ref Line %lf", abs(cur_location.speed_x) + abs(cur_location.speed_y));

  //车道大于可以Diagonal Line
  //获取起点位置3m

  decision_info.header.stamp    = ros::Time::now();
  decision_info.header.frame_id = "route_decision_info";
  decision_info.path_data_REF.clear();

  common_msgs::PathPoint last_ref_point_temp;
  last_ref_point_temp.x = cur_location.pos_x;
  last_ref_point_temp.y = cur_location.pos_y;
  // last_ref_point_temp.v     = 1.0; // 1m/s
  last_ref_point_temp.v = abs(cur_location.speed_x) + abs(cur_location.speed_y); // 1m/s
  if (last_ref_point_temp.v < 0.5)
  {
    last_ref_point_temp.v = 0.5;
  }
  last_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(cur_index_).rtheta;

  //对第一个点赋值
  decision_info.path_data_REF.push_back(last_ref_point_temp);
  ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  -ID %d dir %d-  000 rkappa %lf rdkappa %lf", last_ref_point_temp.x,
           last_ref_point_temp.y, last_ref_point_temp.theta, last_ref_point_temp.v,
           rount_ref_line.REF_line_INFO.at(cur_index_).lane_id,
           rount_ref_line.REF_line_INFO.at(cur_index_).line_direction,
           rount_ref_line.REF_line_INFO.at(cur_index_).rkappa, rount_ref_line.REF_line_INFO.at(cur_index_).rdkappa);

  int old_ref_count = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  generateSurpluslLine(cur_index_, old_ref_count, end_point_index_, last_ref_point_temp, decision_info);

  decision_info.path_plan_valid = 1;
  decision_info.path_mode       = 1;
  route_decision_pub.publish(decision_info);

  ros::Time e_time = ros::Time::now();
  ROS_INFO("generate Diagonal Line %d usetime %0.8lf(s) ,ref point %d", decision_info.path_data_REF.size(),
           e_time.toSec() - b_time.toSec(), end_point_index_ - cur_index_);
  return 1;
}

int generateBackRefLine(int is_right_, int cur_index_, int end_point_index_, double lane_width_)
{
  ros::Time b_time = ros::Time::now();
  ROS_INFO("generate Diagonal Line");

  //车道大于可以Diagonal Line
  //获取起点位置3m
  // plan_msgs::DecisionInfo decision_info;
  decision_info.header.stamp    = ros::Time::now();
  decision_info.header.frame_id = "route_decision_info";
  decision_info.path_data_REF.clear();

  common_msgs::PathPoint last_ref_point_temp;
  last_ref_point_temp.x = cur_location.pos_x;
  last_ref_point_temp.y = cur_location.pos_y;
  last_ref_point_temp.v = abs(cur_location.speed_x) + abs(cur_location.speed_y); // 1m/s
  if (last_ref_point_temp.v < 0.5)
  {
    last_ref_point_temp.v = 0.5;
  }
  last_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(cur_index_).rtheta;

  //对第一个点赋值
  decision_info.path_data_REF.push_back(last_ref_point_temp);
  ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  -ID %d dir %d-  000 rkappa %lf rdkappa %lf", last_ref_point_temp.x,
           last_ref_point_temp.y, last_ref_point_temp.theta, last_ref_point_temp.v,
           rount_ref_line.REF_line_INFO.at(cur_index_).lane_id,
           rount_ref_line.REF_line_INFO.at(cur_index_).line_direction,
           rount_ref_line.REF_line_INFO.at(cur_index_).rkappa, rount_ref_line.REF_line_INFO.at(cur_index_).rdkappa);

  int last_ref_index      = cur_index_;
  double radian_          = rount_ref_line.REF_line_INFO.at(cur_index_).rtheta * M_PI / 180;
  double target_speed     = 1.0; // 1m/s-fr
  double max_target_angle = 30.0;
  int cur_lane_index      = 0; //根据当前位置判断在第几个车道上，或者考虑按照宽度平移，待定
  // double diagonal_distance_x = right_;
  double diagonal_distance_x = 0 - lane_width_;
  double diagonal_distance_y = getDiagonalDistanceY(max_target_angle, lane_width_);
  int old_ref_count          = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  // int old_ref_count = end_point_index_;

  if (is_right_ > 0)
  {
    max_target_angle = 0 - max_target_angle;
  }

  last_ref_index = generateParallelLine(max_target_angle, diagonal_distance_x, 1.5, radian_, last_ref_index,
                                        old_ref_count, end_point_index_, last_ref_point_temp, decision_info);
  last_ref_index =
      generateDiagonalLineBack(max_target_angle, diagonal_distance_x, diagonal_distance_y, radian_, last_ref_index,
                               old_ref_count, end_point_index_, last_ref_point_temp, decision_info);

  generateSurpluslLine(last_ref_index, old_ref_count, end_point_index_, last_ref_point_temp, decision_info);

  decision_info.path_plan_valid = 1;
  decision_info.path_mode       = 1;
  route_decision_pub.publish(decision_info);

  ros::Time e_time = ros::Time::now();
  ROS_INFO("generate Diagonal Line %d usetime %0.8lf(s) ,ref point %d", decision_info.path_data_REF.size(),
           e_time.toSec() - b_time.toSec(), old_ref_count - cur_index_, lane_width_);
  return 1;
}

int generateFirstNewRefLine(int is_right_, int cur_index_, double max_ds, int end_point_index_, double lane_width_,
                            double left_, double right_)
{

  ros::Time b_time = ros::Time::now();
  ROS_INFO("generate Diagonal Line");

  //车道大于可以Diagonal Line
  //获取起点位置3m
  // plan_msgs::DecisionInfo decision_info;

  decision_info.header.stamp    = ros::Time::now();
  decision_info.header.frame_id = "route_decision_info";
  decision_info.path_data_REF.clear();

  common_msgs::PathPoint last_ref_point_temp;
  last_ref_point_temp.x = cur_location.pos_x;
  last_ref_point_temp.y = cur_location.pos_y;
  last_ref_point_temp.v = abs(cur_location.speed_x) + abs(cur_location.speed_y); // 1m/s
  if (last_ref_point_temp.v < 0.5)
  {
    last_ref_point_temp.v = 0.5;
  }
  last_ref_point_temp.theta = rount_ref_line.REF_line_INFO.at(cur_index_).rtheta;

  //对第一个点赋值
  decision_info.path_data_REF.push_back(last_ref_point_temp);
  ROS_INFO("new Line (%lf,%lf) theta:%lf speed:%lf  -ID %d dir %d-  000 rkappa %lf rdkappa %lf", last_ref_point_temp.x,
           last_ref_point_temp.y, last_ref_point_temp.theta, last_ref_point_temp.v,
           rount_ref_line.REF_line_INFO.at(cur_index_).lane_id,
           rount_ref_line.REF_line_INFO.at(cur_index_).line_direction,
           rount_ref_line.REF_line_INFO.at(cur_index_).rkappa, rount_ref_line.REF_line_INFO.at(cur_index_).rdkappa);

  int last_ref_index      = cur_index_;
  double radian_          = rount_ref_line.REF_line_INFO.at(cur_index_).rtheta * M_PI / 180;
  double target_speed     = 1.0; // 1m/s-fr
  double max_target_angle = 30.0;
  int cur_lane_index      = 0; //根据当前位置判断在第几个车道上，或者考虑按照宽度平移，待定
  // double diagonal_distance_x = right_;
  double diagonal_distance_x = 0 - lane_width_;
  double diagonal_distance_y = getDiagonalDistanceY(max_target_angle, lane_width_);
  int old_ref_count          = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  // int old_ref_count = end_point_index_;

  if (is_right_ > 0)
  {
    max_target_angle = 0 - max_target_angle;
  }

  //平移
  last_ref_index = generateParallelLine(max_target_angle, diagonal_distance_x, max_ds, radian_, last_ref_index,
                                        old_ref_count, end_point_index_, last_ref_point_temp, decision_info);

  last_ref_index =
      generateDiagonalLineBack(max_target_angle, diagonal_distance_x, diagonal_distance_y, radian_, last_ref_index,
                               old_ref_count, end_point_index_, last_ref_point_temp, decision_info);

  generateSurpluslLine(last_ref_index, old_ref_count, end_point_index_, last_ref_point_temp, decision_info);

  decision_info.path_plan_valid = 1;
  decision_info.path_mode       = 1;
  route_decision_pub.publish(decision_info);

  ros::Time e_time = ros::Time::now();
  ROS_INFO("generate Diagonal Line %d usetime %0.8lf(s) ,ole ref point %d diagonal_distance_x %lf l:%lf r%lf",
           decision_info.path_data_REF.size(), e_time.toSec() - b_time.toSec(), old_ref_count - cur_index_, lane_width_,
           left_, right_);
  return 1;
}

void detectionObsJudge()
{
  int ans = getLocationFromVectorTemp(lidar_location_obs_time);
  if (ans == 1)
  {
    if (obstacle_detection_judge_tip == 0)
    {
      obstacle_detection_vec.clear();
    }

    // rvizPointPub(lidar_location_obs_time.pos_x, lidar_location_obs_time.pos_y);
    // ROS_INFO("lidar_location_temp (%lf,%lf) heading %lf", lidar_location_temp.pos_x, lidar_location_temp.pos_y,
    //          lidar_location_temp.heading);

    visualization_msgs::MarkerArray maker_temp;
    visualization_msgs::Marker points;

    //清除rviz显示
    lidarRvizPusClear(points, 0);
    maker_temp.markers.push_back(points);
    obstacle_cam_lidar_pub.publish(maker_temp);
    maker_temp.markers.clear();

    //障碍物判断初值
    int id_index   = 0;
    double radian_ = lidar_location_obs_time.heading * M_PI / 180;
    double x_      = lidar_location_obs_time.pos_x;
    double y_      = lidar_location_obs_time.pos_y;
    double temp_x_ = 0;
    double temp_y_ = 0;
    //更新查询参考线位置
    int ref_index = cur_location.ref_index;
    if (ref_index > 0 && ref_index < (static_cast< int >(rount_ref_line.REF_line_INFO.size()) - 2))
    {
      int obs_num = static_cast< int >(lidar_detection_obs_vec.size());
      for (int i = 0; i < obs_num; i++)
      {
        ObstacleDetectionTemp obstacle_temp;

        obstacle_temp.ref_index = ref_index;
        obstacle_temp.obs_id    = lidar_detection_obs_vec.at(i).id;
        obstacle_temp.max_x     = 0 - lidar_detection_obs_vec.at(i).state[0];
        obstacle_temp.min_y     = lidar_detection_obs_vec.at(i).state[1];
        obstacle_temp.min_x     = 0 - lidar_detection_obs_vec.at(i).state[2];
        obstacle_temp.max_y     = lidar_detection_obs_vec.at(i).state[3];

        // if (obstacle_temp.min_y > 0.0 && obstacle_temp.min_y < 60.0 &&
        //     min(abs(obstacle_temp.max_x), abs(obstacle_temp.min_x)) < 20.0)
        if (abs(obstacle_temp.min_y) > 0.0)
        {
          // showRefBoundary( obstacle_temp.ref_index,0);
          //显示当前点在参考线路径边界上的位置
          id_index++;
          visualization_msgs::Marker points;

          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[0]);

          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[1]);

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[2]);

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[3]);

          // obstacle_temp.judge_tip = findMinDistaceObsAndRef(
          //     obstacle_temp.ref_index, obstacle_temp.corner_point[0], obstacle_temp.corner_point[1],
          //     obstacle_temp.corner_point[2], obstacle_temp.corner_point[3], obstacle_temp);
          int judge_tip = findMinDistaceObsAndRef(obstacle_temp);
          // int judge_tip = 0;
          // temp_x_ = min_x;
          // temp_y_ = min_y;
          // doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, p);

          if (judge_tip > 0)
          {
            ROS_WARN("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
                     obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
            obstacle_detection_vec.push_back(obstacle_temp);
            lidarRvizPusInit(points, id_index, 1, 0, 0);
            points.points.push_back(obstacle_temp.corner_point[0]);
            points.points.push_back(obstacle_temp.corner_point[1]);
            points.points.push_back(obstacle_temp.corner_point[2]);
            points.points.push_back(obstacle_temp.corner_point[3]);
            points.points.push_back(obstacle_temp.corner_point[0]);
            maker_temp.markers.push_back(points);
          }
          else
          {
            // ROS_INFO("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
            //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
            lidarRvizPusInit(points, id_index, 0, 0, 0);
            points.points.push_back(obstacle_temp.corner_point[0]);
            points.points.push_back(obstacle_temp.corner_point[1]);
            points.points.push_back(obstacle_temp.corner_point[2]);
            points.points.push_back(obstacle_temp.corner_point[3]);
            points.points.push_back(obstacle_temp.corner_point[0]);
            maker_temp.markers.push_back(points);
          }
        }
      }
      obstacle_cam_lidar_pub.publish(maker_temp);
      if (!obstacle_detection_vec.empty())
      {
        obstacle_detection_judge_tip = 1;
      }
    }
  }
  else
  {
    ROS_INFO("Can not find lidar_location_temp");
  }
  //判断耗时统计
  // ros::Time lidar_out = ros::Time::now();
  // if (max_duration < (lidar_out.toSec() - lidar_in.toSec()))
  // {
  //   max_duration = lidar_out.toSec() - lidar_in.toSec();
  // }
  // ROS_ERROR("Lidar Judge use %0.8lf(s) MAX Dur is %0.8lf(s)", lidar_out.toSec() - lidar_in.toSec(), max_duration);
}

void detectionCameraObsJudge()
{
  int ans = 1;
  // int ans = getLocationFromVectorTemp(camera_location_obs_time);
  if (ans == 1)
  {
    visualization_msgs::MarkerArray maker_temp;
    visualization_msgs::Marker points;

    //清除rviz显示
    camRvizPusClear(points, 0);
    maker_temp.markers.push_back(points);
    obstacle_camera_pub.publish(maker_temp);
    maker_temp.markers.clear();

    //障碍物判断初值
    int id_index = 0;
    //临时使用实时定位数据0925
    double radian_ = cur_location.heading * M_PI / 180;
    double x_      = cur_location.pos_x;
    double y_      = cur_location.pos_y;
    // double radian_ = camera_location_obs_time.heading * M_PI / 180;
    // double x_      = camera_location_obs_time.pos_x;
    // double y_      = camera_location_obs_time.pos_y;
    double temp_x_ = 0;
    double temp_y_ = 0;
    int ref_index  = cur_location.ref_index;
    if (ref_index > 0 && ref_index < (static_cast< int >(rount_ref_line.REF_line_INFO.size()) - 2))
    {
      int obs_num = static_cast< int >(camera_detection_obs_vec.size());
      std::vector< common_msgs::DetectionInfo >::iterator camera_it;
      for (camera_it = camera_detection_obs_vec.begin(); camera_it != camera_detection_obs_vec.end(); camera_it++)
      {
        ObstacleDetectionTemp obstacle_temp;

        obstacle_temp.ref_index = ref_index;
        obstacle_temp.obs_id    = camera_it->id;
        obstacle_temp.min_x     = camera_it->state[0];
        obstacle_temp.min_y     = camera_it->state[1];
        obstacle_temp.max_x     = camera_it->state[2];
        obstacle_temp.max_y     = camera_it->state[3];

        double judge_y_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
        double judge_x_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
        if (judge_y_temp < 60.0)
        {

          id_index++;
          visualization_msgs::Marker points;
          //转换全局
          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[0]);

          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[1]);

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[2]);

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[3]);

          int judge_tip = 1;

          //显示和赋值
          if (judge_tip > 0)
          {
            // ROS_WARN("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
            //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
            camRvizPusInit(points, id_index, 0, 0, 1);
            points.points.push_back(obstacle_temp.corner_point[0]);
            points.points.push_back(obstacle_temp.corner_point[1]);
            points.points.push_back(obstacle_temp.corner_point[2]);
            points.points.push_back(obstacle_temp.corner_point[3]);
            points.points.push_back(obstacle_temp.corner_point[0]);
            maker_temp.markers.push_back(points);
          }
          else
          {
            // ROS_INFO("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
            //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
            camRvizPusInit(points, id_index, 1, 0, 1);
            points.points.push_back(obstacle_temp.corner_point[0]);
            points.points.push_back(obstacle_temp.corner_point[1]);
            points.points.push_back(obstacle_temp.corner_point[2]);
            points.points.push_back(obstacle_temp.corner_point[3]);
            points.points.push_back(obstacle_temp.corner_point[0]);
            maker_temp.markers.push_back(points);
          }
        }
      }
      obstacle_camera_pub.publish(maker_temp);
    }
  }
  else
  {
    ROS_INFO("Can not find camera_location_temp");
  }
}

void detectionLidarObsJudge()
{
  int ans = getLocationFromVectorTemp(lidar_location_obs_time);
  if (ans == 1)
  {
    if (obstacle_detection_judge_tip == 0)
    {
      obstacle_detection_vec.clear();
    }

    // rvizPointPub(lidar_location_obs_time.pos_x, lidar_location_obs_time.pos_y);
    // ROS_INFO("lidar_location_temp (%lf,%lf) heading %lf", lidar_location_obs_time.pos_x,
    // lidar_location_obs_time.pos_y,
    //          lidar_location_obs_time.heading);

    visualization_msgs::MarkerArray maker_temp;
    visualization_msgs::Marker points;

    //清除rviz显示
    lidarRvizPusClear(points, 0);
    maker_temp.markers.push_back(points);
    obstacle_cam_lidar_pub.publish(maker_temp);
    maker_temp.markers.clear();

    //障碍物判断初值
    int id_index = 0;
    // double radian_ = lidar_location_obs_time.heading * M_PI / 180;
    // double x_      = lidar_location_obs_time.pos_x;
    // double y_      = lidar_location_obs_time.pos_y;
    //临时使用实时定位数据0925
    double radian_ = cur_location.heading * M_PI / 180;
    double x_      = cur_location.pos_x;
    double y_      = cur_location.pos_y;
    double temp_x_ = 0;
    double temp_y_ = 0;
    //更新查询参考线位置
    int ref_index = cur_location.ref_index;
    if (ref_index > 0 && ref_index < (static_cast< int >(rount_ref_line.REF_line_INFO.size()) - 2))
    {
      int obs_num = static_cast< int >(lidar_detection_obs_vec.size());
      std::vector< common_msgs::DetectionInfo >::iterator lidar_it;
      for (lidar_it = lidar_detection_obs_vec.begin(); lidar_it != lidar_detection_obs_vec.end(); lidar_it++)
      {
        ObstacleDetectionTemp obstacle_temp;

        obstacle_temp.ref_index = ref_index;
        lidar_it->state[0];
        obstacle_temp.obs_id = lidar_it->id;
        obstacle_temp.max_x  = 0 - lidar_it->state[0];
        obstacle_temp.min_y  = lidar_it->state[1];
        obstacle_temp.min_x  = 0 - lidar_it->state[2];
        obstacle_temp.max_y  = lidar_it->state[3];

        // obstacle_temp.max_x = 0 - max(max(lidar_it->peek.at(0).x, lidar_it->peek.at(1).x),
        //                               max(lidar_it->peek.at(2).x, lidar_it->peek.at(3).x));
        // obstacle_temp.min_x = 0 - min(min(lidar_it->peek.at(0).x, lidar_it->peek.at(1).x),
        //                               min(lidar_it->peek.at(2).x, lidar_it->peek.at(3).x));
        // obstacle_temp.max_y = max(max(lidar_it->peek.at(0).y, lidar_it->peek.at(1).y),
        //                           max(lidar_it->peek.at(2).y, lidar_it->peek.at(3).y));
        // obstacle_temp.min_y = min(min(lidar_it->peek.at(0).y, lidar_it->peek.at(1).y),
        //                           min(lidar_it->peek.at(2).y, lidar_it->peek.at(3).y));

        // for (int i = 0; i < obs_num; i++)
        // {
        // ObstacleDetectionTemp obstacle_temp;

        // obstacle_temp.ref_index = ref_index;
        // obstacle_temp.obs_id    = lidar_detection_obs_vec.at(i).id;
        // obstacle_temp.max_x     = 0 - lidar_detection_obs_vec.at(i).state[0];
        // obstacle_temp.min_y     = lidar_detection_obs_vec.at(i).state[1];
        // obstacle_temp.min_x     = 0 - lidar_detection_obs_vec.at(i).state[2];
        // obstacle_temp.max_y     = lidar_detection_obs_vec.at(i).state[3];
        double judge_y_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
        double judge_x_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
        if (judge_y_temp < 60.0)
        // if (judge_y_temp > -30 && judge_y_temp < 60.0 && judge_x_temp < 20.0)
        // if (obstacle_temp.min_y > -20 && obstacle_temp.min_y < 60.0 &&
        //     min(abs(obstacle_temp.max_x), abs(obstacle_temp.min_x)) < 20.0)
        // if (obstacle_temp.min_y > 0 && obstacle_temp.min_y < 70.0 &&
        //     min(abs(obstacle_temp.max_x), abs(obstacle_temp.min_x)) < 25.0)
        // if (abs(obstacle_temp.min_y) > 0.0)
        // if (abs(obstacle_temp.min_y) > -100.0)
        {
          // showRefBoundary( obstacle_temp.ref_index,0);
          //显示当前点在参考线路径边界上的位置
          id_index++;
          visualization_msgs::Marker points;
          //转换全局
          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[0]);

          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[1]);

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[2]);

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[3]);

          // temp_x_ = 0 - lidar_it->peek.at(0).x;
          // temp_y_ = lidar_it->peek.at(0).y;
          // doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[0]);

          // temp_x_ = 0 - lidar_it->peek.at(1).x;
          // temp_y_ = lidar_it->peek.at(1).y;
          // doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[1]);

          // temp_x_ = 0 - lidar_it->peek.at(2).x;
          // temp_y_ = lidar_it->peek.at(2).y;
          // doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[2]);

          // temp_x_ = 0 - lidar_it->peek.at(3).x;
          // temp_y_ = lidar_it->peek.at(3).y;
          // doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, obstacle_temp.corner_point[3]);

          //判断碰撞
          //          int judge_tip = isObsduringRef(obstacle_temp);
          int judge_tip = 0;
          // if (abs(obstacle_temp.min_y) < 40 && min(abs(obstacle_temp.max_x), abs(obstacle_temp.min_x)) < 2.5)
          if (obstacle_temp.min_y > 9 && obstacle_temp.min_y < 40 &&
              min(abs(obstacle_temp.max_x), abs(obstacle_temp.min_x)) < 2.1)
          {
            judge_tip = 0;
            obstacle_detection_vec.push_back(obstacle_temp);
          }

          if (obstacle_temp.max_x <= -2.0 && obstacle_temp.max_x >= -6.0)
          {
            if (obstacle_temp.min_y > 0 && obstacle_temp.min_y < 40)
            {
              judge_tip = 1;
              obstacle_detection_right_vec.push_back(obstacle_temp);
            }
            else if (obstacle_temp.min_y < 0 && obstacle_temp.max_y > 0)
            {
              judge_tip = 1;
              obstacle_detection_right_vec.push_back(obstacle_temp);
            }
            else if (obstacle_temp.max_y < 0 && obstacle_temp.max_y > -40)
            {
              judge_tip = 1;
              obstacle_detection_right_vec.push_back(obstacle_temp);
            }
            else
            {
              /* code for False */
            }
          }

          if (obstacle_temp.max_x <= 6.0 && obstacle_temp.max_x >= 2.0)
          {
            if (obstacle_temp.min_y > 0 && obstacle_temp.min_y < 40)
            {
              judge_tip = 1;
              obstacle_detection_left_vec.push_back(obstacle_temp);
            }
            else if (obstacle_temp.min_y < 0 && obstacle_temp.max_y > 0)
            {
              judge_tip = 1;
              obstacle_detection_left_vec.push_back(obstacle_temp);
            }
            else if (obstacle_temp.max_y < 0 && obstacle_temp.max_y > -40)
            {
              judge_tip = 1;
              obstacle_detection_left_vec.push_back(obstacle_temp);
            }
            else
            {
              /* code for False */
            }
          }

          // if (obstacle_temp.min_y > -19 && obstacle_temp.min_y < 40 && obstacle_temp.min_x >= 2.0 &&
          //     obstacle_temp.min_x <= 6.0)
          // {
          //   judge_tip = 0;
          //   obstacle_detection_left_vec.push_back(obstacle_temp);
          // }
          if (obstacle_temp.min_y > 9 && obstacle_temp.min_y < 40 &&
              min(abs(obstacle_temp.max_x), abs(obstacle_temp.min_x)) < 20)
          {
            judge_tip = 0;
            obstacle_detection_front_vec.push_back(obstacle_temp);
          }

          //显示和赋值
          if (judge_tip > 0)
          {
            // ROS_WARN("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
            //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
            lidarRvizPusInit(points, id_index, 1, 0, 0);
            points.points.push_back(obstacle_temp.corner_point[0]);
            points.points.push_back(obstacle_temp.corner_point[1]);
            points.points.push_back(obstacle_temp.corner_point[2]);
            points.points.push_back(obstacle_temp.corner_point[3]);
            points.points.push_back(obstacle_temp.corner_point[0]);
            maker_temp.markers.push_back(points);
          }
          else
          {
            // ROS_INFO("The obs Y distace agv is (%lf,%lf) (%lf,%lf) %d", obstacle_temp.min_x, obstacle_temp.min_y,
            //          obstacle_temp.max_x, obstacle_temp.max_y, judge_tip);
            lidarRvizPusInit(points, id_index, 0, 0, 0);
            points.points.push_back(obstacle_temp.corner_point[0]);
            points.points.push_back(obstacle_temp.corner_point[1]);
            points.points.push_back(obstacle_temp.corner_point[2]);
            points.points.push_back(obstacle_temp.corner_point[3]);
            points.points.push_back(obstacle_temp.corner_point[0]);
            maker_temp.markers.push_back(points);
          }
        }
      }
      obstacle_cam_lidar_pub.publish(maker_temp);
      if (!obstacle_detection_vec.empty())
      {
        obstacle_detection_judge_tip = 1;
      }
    }
  }
  else
  {
    ROS_INFO("Can not find lidar_location_temp");
  }
  //判断耗时统计
  // ros::Time lidar_out = ros::Time::now();
  // if (max_duration < (lidar_out.toSec() - lidar_in.toSec()))
  // {
  //   max_duration = lidar_out.toSec() - lidar_in.toSec();
  // }
  // ROS_ERROR("Lidar Judge use %0.8lf(s) MAX Dur is %0.8lf(s)", lidar_out.toSec() - lidar_in.toSec(), max_duration);
}

double mathLanelength(int cur_index_, int end_index_)
{
  int begin_ref_index_ = cur_index_;
  double delta_rs      = 0;
  for (int i = begin_ref_index_; i < end_index_; i++)
  {
    if (rount_ref_line.REF_line_INFO.at(i).rkappa == 0 && rount_ref_line.REF_line_INFO.at(i).rdkappa == 0)
    {
      delta_rs = rount_ref_line.REF_line_INFO.at(i).rs - rount_ref_line.REF_line_INFO.at(cur_index_).rs;
    }
    else
    {
      return delta_rs;
    }
  }
  return delta_rs;
}

int findEndPointRef(int cur_index_)
{
  int begin_ref_index_ = cur_index_;
  int count            = static_cast< int >(rount_ref_line.REF_line_INFO.size());
  int end_point_index_ = begin_ref_index_;

  for (int i = begin_ref_index_; i < count; i++)
  {

    double end_point_distance = mathPointDistanceSquare(
        rount_ref_line.REF_line_INFO.at(i).rx, rount_ref_line.REF_line_INFO.at(i).ry, task_click.x, task_click.y);
    if (end_point_distance < (0.25 * 0.25))
    {
      end_point_index_ = i;
    }
  }
  return end_point_index_;
}

void publishTargetPointPlanningMsg(int path_mode_)
{
  plan_msgs::DecisionInfo decision_info_;

  decision_info_.relative_position.x = obu_agv_distance.x;
  decision_info_.relative_position.y = obu_agv_distance.y;
  decision_info_.relative_position.z = obu_agv_distance.z;
  decision_info_.CMD_estop           = 0;
  decision_info_.CMD_gear            = 0;
  decision_info_.CMD_hand_brake      = 0;
  decision_info_.CMD_lift            = 0;
  decision_info_.path_plan_valid     = 1;
  decision_info_.move_permit         = 1;          // default
  decision_info_.path_mode           = path_mode_; // lidar location
  route_decision_pub.publish(decision_info_);
  ROS_ERROR("Planning:I publish relative position x=%f,y=%f,z=%f", decision_info_.relative_position.x,
            decision_info_.relative_position.y, decision_info_.relative_position.z);
}

void fixedLidarInfoCallback(const location_sensor_msgs::FixedLidarInfo::ConstPtr &fixed_lidar_msg)
{
  // ROS_INFO("OBU:I received a lidar location.");

  obu_agv_distance.x = fixed_lidar_msg->agv_distance.x;
  // obu_agv_distance.y = fixed_lidar_msg->agv_distance.y; //临时处理等东博修改后更新回来
  // obu_agv_distance.y = 0 - fixed_lidar_msg->agv_distance.y; //临时处理等东博修改后更新回来
  obu_agv_distance.y = fixed_lidar_msg->agv_distance.y; //临时处理等东博修改后更新回来
  obu_agv_distance.z = fixed_lidar_msg->agv_distance.z;
  //终点判断，启动obu
  // if (enable_fixed_lidar == 1)
  // if (abs(obu_agv_distance.y) < 3.0 && abs(obu_agv_distance.x) < 19.0)
  if (abs(obu_agv_distance.y) < 3.0 && enable_fixed_lidar == 1)
  {
    publishTargetPointPlanningMsg(4);
    // ROS_ERROR("OBU:Lidar location agv distance x= %f,agv distance y= %f,agv distance z= %f", obu_agv_distance.x,
    //           obu_agv_distance.y, obu_agv_distance.z);
  }
  else
  {
    // publishTargetPointPlanningMsg(1);
  }
}

void doObsLocationAndMatching(std::vector< ObstacleDetectionTemp > &detection_vec,
                              std::vector< ObstacleJudgeStr > &obs_judge_vec, int pos_)
{
  int count_obs_judge_vec = static_cast< int >(obs_judge_vec.size());
  if (count_obs_judge_vec > 0)
  {
    // match
  }
  else
  {
    // copy
    for (std::vector< ObstacleDetectionTemp >::iterator it_vec = detection_vec.begin(); it_vec != detection_vec.end();
         it_vec++)
    {
      ObstacleJudgeStr obs_judge_temp;
      obs_judge_temp.agv_ref_index = it_vec->ref_index;

      ObsPos obs_pos_temp;
      obs_pos_temp.id             = 0;
      obs_pos_temp.relative_pos.x = it_vec->min_x;
      obs_pos_temp.relative_pos.y = it_vec->min_y;
      obs_pos_temp.map_pos        = it_vec->corner_point[0];
      obs_judge_temp.pos.push_back(obs_pos_temp);

      obs_pos_temp.id             = 1;
      obs_pos_temp.relative_pos.x = it_vec->min_x;
      obs_pos_temp.relative_pos.y = it_vec->max_y;
      obs_pos_temp.map_pos        = it_vec->corner_point[1];
      obs_judge_temp.pos.push_back(obs_pos_temp);

      obs_pos_temp.id             = 2;
      obs_pos_temp.relative_pos.x = it_vec->max_x;
      obs_pos_temp.relative_pos.y = it_vec->max_y;
      obs_pos_temp.map_pos        = it_vec->corner_point[2];
      obs_judge_temp.pos.push_back(obs_pos_temp);

      obs_pos_temp.id             = 3;
      obs_pos_temp.relative_pos.x = it_vec->max_x;
      obs_pos_temp.relative_pos.y = it_vec->min_y;
      obs_pos_temp.map_pos        = it_vec->corner_point[3];
      obs_judge_temp.pos.push_back(obs_pos_temp);

      switch (pos_)
      {
      case 1:
        obs_judge_temp.is_front = 1;
        break;
      case 2:
        obs_judge_temp.is_left = 1;
        break;
      case 3:
        obs_judge_temp.is_right = 1;
        break;
      case 4:
        obs_judge_temp.is_behind = 1;
        break;
      default:
        break;
      }
    }
  }
}

void obsHanding()
{
  // std::vector< ObstacleDetectionTemp >
  // std::vector< ObstacleJudgeStr >

  obstacle_detection_vec;
  obstacle_detection_left_vec;
  obstacle_detection_right_vec;
  obstacle_detection_front_vec;
  obstacle_around_agv_vec;

  int obs_agv_pos = 0;

  int count_obstacle_vec = static_cast< int >(obstacle_detection_vec.size());
  if (count_obstacle_vec > 0)
  {
    obs_agv_pos = 1;
    doObsLocationAndMatching(obstacle_detection_vec, obstacle_around_agv_vec, obs_agv_pos);
  }

  // clear obstacle_around_agv_vec
}

void controlAgvPub(double speed_, double stop_)
{
  control_msgs::ADControlAGV control_agv_temp;
  control_agv_temp.Vel_Req = speed_;
  control_agv_temp.EStop   = stop_;
  agv_control_pub.publish(control_agv_temp);
  if (speed_ < 1.0)
  {
    // ROS_ERROR("control agv message: speed %lf stop tip %lf", speed_, stop_);
    ROS_WARN("control agv message: speed %lf stop tip %lf", speed_, stop_);
  }
  else
  {
    // ROS_INFO("control agv message: speed %lf stop tip %lf", speed_, stop_);
  }
}

void newBoundyRvizPusInit(visualization_msgs::Marker &points, int id_index, double color_r, double color_g,
                          double color_b)
{
  points.header.frame_id = "/odom";
  points.header.stamp    = ros::Time::now();
  points.ns              = "/per/new_boundy_show";
  // points.action             = visualization_msgs::Marker::ADD;
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = id_index;
  points.type               = visualization_msgs::Marker::LINE_STRIP;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.3;
  points.scale.y = 0.3;
  // Points are green
  points.color.r = color_r;
  points.color.g = color_g;
  points.color.b = color_b;
  points.color.a = 1.0;
}

void newLineBoundyPub()
{
  visualization_msgs::MarkerArray maker_temp;
  visualization_msgs::Marker points;
  geometry_msgs::Point p;
  int id_temp = 0;

  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.748841166]: 107-108
  p.x = -412.159841;
  p.y = 230.044202;
  p.z = -1.474400;
  points.points.push_back(p);
  p.x = -34.064967;
  p.y = 263.050593;
  p.z = -1.352700;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.749102510]: 106-109
  p.x = -412.172634;
  p.y = 226.372597;
  p.z = -1.474400;
  points.points.push_back(p);
  p.x = -33.765750;
  p.y = 259.301397;
  p.z = -1.339200;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.749250029]: 105-110
  p.x = -411.429507;
  p.y = 222.589683;
  p.z = -1.389100;
  points.points.push_back(p);
  p.x = -33.457030;
  p.y = 255.573239;
  p.z = -1.265900;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.749396740]: 104-111
  p.x = -411.065913;
  p.y = 218.681674;
  p.z = -1.357400;
  points.points.push_back(p);
  p.x = -33.117883;
  p.y = 251.644260;
  p.z = -1.221500;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.749564965]: 103-112
  p.x = -410.712475;
  p.y = 214.835801;
  p.z = -1.338900;
  points.points.push_back(p);
  p.x = -32.725617;
  p.y = 247.804921;
  p.z = -1.290500;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.749745791]: 102-113
  p.x = -410.366329;
  p.y = 211.062033;
  p.z = -1.442500;
  points.points.push_back(p);
  p.x = -32.361205;
  p.y = 244.092096;
  p.z = -1.345700;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.749931779]: 101-114
  p.x = -410.022905;
  p.y = 207.319325;
  p.z = -1.488000;
  points.points.push_back(p);
  p.x = -32.079839;
  p.y = 240.361814;
  p.z = -1.402800;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.750803959]: 204-205
  p.x = -431.877011;
  p.y = 350.847114;
  p.z = -1.942500;
  points.points.push_back(p);
  p.x = -5.196650;
  p.y = 388.038893;
  p.z = -2.096800;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.750972369]: 203-206
  p.x = -431.409062;
  p.y = 346.580557;
  p.z = -1.970500;
  points.points.push_back(p);
  p.x = -4.815169;
  p.y = 383.804814;
  p.z = -2.068400;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.751135800]: 202-207
  p.x = -430.773135;
  p.y = 341.537167;
  p.z = -1.891800;
  points.points.push_back(p);
  p.x = -4.351877;
  p.y = 378.682206;
  p.z = -2.068400;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.751315918]: 201-208
  p.x = -430.692049;
  p.y = 337.205424;
  p.z = -1.878700;
  points.points.push_back(p);
  p.x = -4.006236;
  p.y = 374.446033;
  p.z = -2.039200;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.751490306]: 301
  p.x = -0.019864;
  p.y = 384.378246;
  p.z = -2.090700;
  points.points.push_back(p);
  p.x = 8.999942;
  p.y = 281.089699;
  p.z = -1.520000;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.751675928]: 302
  p.x = 3.737311;
  p.y = 384.714619;
  p.z = -1.987900;
  points.points.push_back(p);
  p.x = 12.730865;
  p.y = 281.459434;
  p.z = -1.547500;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.751806800]: 303
  p.x = 7.475519;
  p.y = 385.020008;
  p.z = -1.904500;
  points.points.push_back(p);
  p.x = 16.488983;
  p.y = 281.784719;
  p.z = -1.493500;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.751925307]: 304
  p.x = 11.603457;
  p.y = 385.402790;
  p.z = -1.910900;
  points.points.push_back(p);
  p.x = 20.623479;
  p.y = 282.143087;
  p.z = -1.424000;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.752043522]: 305
  p.x = 15.779395;
  p.y = 385.758795;
  p.z = -1.901100;
  points.points.push_back(p);
  p.x = 24.778718;
  p.y = 282.500276;
  p.z = -1.473200;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.752157895]: 306
  p.x = 19.532744;
  p.y = 386.079662;
  p.z = -1.933300;
  points.points.push_back(p);
  p.x = 28.521727;
  p.y = 282.820072;
  p.z = -1.476600;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.752282124]: 307
  p.x = 23.271989;
  p.y = 386.412776;
  p.z = -1.971100;
  points.points.push_back(p);
  p.x = 32.261876;
  p.y = 283.131009;
  p.z = -1.496700;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.752409350]: 401
  p.x = -225.318734;
  p.y = 211.044416;
  p.z = -1.464200;
  points.points.push_back(p);
  p.x = -212.162310;
  p.y = 59.771493;
  p.z = -0.182300;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.752535288]: 402
  p.x = -221.601078;
  p.y = 211.385278;
  p.z = -1.370000;
  points.points.push_back(p);
  p.x = -208.428552;
  p.y = 60.116741;
  p.z = -0.143800;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.752656543]: 403
  p.x = -217.682514;
  p.y = 211.738753;
  p.z = -1.359700;
  points.points.push_back(p);
  p.x = -204.511796;
  p.y = 60.475772;
  p.z = -0.193100;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);
  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0, 0, 0);
  points.points.clear();
  // [ INFO] [1569566757.752772584]: 404
  p.x = -213.858236;
  p.y = 212.096992;
  p.z = -1.358200;
  points.points.push_back(p);
  p.x = -200.658539;
  p.y = 60.744093;
  p.z = -0.175900;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);

  id_temp++;
  newBoundyRvizPusInit(points, id_temp, 0.9, 0.9, 0);
  points.points.clear();
  // [ INFO] [1569566757.752891799]: 405
  p.x = -210.082317;
  p.y = 212.378883;
  p.z = -1.384900;
  points.points.push_back(p);
  p.x = -196.921382;
  p.y = 60.981767;
  p.z = -0.145900;
  points.points.push_back(p);
  maker_temp.markers.push_back(points);

  rviz_new_boundary_pub_.publish(maker_temp);
  ROS_INFO("Pun new boundary %d", maker_temp.markers.size());
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "per_obs");
  ros::NodeHandle n;

  //设置参数
  GLogHelper gh(argv[0]);

  fusion_location_sub = n.subscribe("/localization/fusion_msg", 5, recvFusionLocationCallback);
  fusion_obstacle_sub = n.subscribe("/perception/obstacle_info", 5, recvFusionObuCallback);
  cam_sub             = n.subscribe("/drivers/perception/camera_obstacle_info", 5, recvCamObsCallback);
  lidar_sub           = n.subscribe("/perception/detection_lidar", 5, recvLidarObsCallback);
  // lidar_sub      = n.subscribe("/perception/detection_lidar_test", 5, recvLidarObsCallback);
  ref_line_sub   = n.subscribe("/map/ref_point_info", 5, recvRefCallback);
  task_click_sub = n.subscribe("/monitor/rviz_click_lane", 5, recvClickCallback);
  lidar_obu_sub  = n.subscribe("/drivers/localization/fixed_lidar_msg", 5, fixedLidarInfoCallback);
  agv_status_sub = n.subscribe("/drivers/com2agv/agv_status", 5, agvStatusCallback);

  // route_decision_pub = n.advertise< plan_msgs::DecisionInfo >("/plan/decision_info", 10, true);
  route_decision_pub = n.advertise< plan_msgs::DecisionInfo >("/map/decision_info", 10, true);
  agv_control_pub    = n.advertise< control_msgs::ADControlAGV >("/map/stop_msg", 10, true);
  obstacle_point_pub = n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_msg", 5, true);
  agv_point_pub      = n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_msg", 5, true);
  obstacle_cam_lidar_pub =
      n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_cam_lidar_msg", 5, true);
  obstacle_camera_pub = n.advertise< visualization_msgs::MarkerArray >("/perception/rviz_obstacle_camera_msg", 5, true);

  rviz_point_pub_          = n.advertise< geometry_msgs::PointStamped >("/obs/veh_point_msg", 5, true);
  rviz_boundary_point_pub_ = n.advertise< geometry_msgs::PointStamped >("/obs/veh_point_boundary_msg", 5, true);
  rviz_left_boundary_point_pub_ =
      n.advertise< geometry_msgs::PointStamped >("/obs/veh_left_point_boundary_msg", 5, true);

  rviz_new_boundary_pub_ = n.advertise< visualization_msgs::MarkerArray >("/obs/new_boundary_msg", 5, true);

  // newLineBoundyPub();

  int count = 0;

  double last_min_obs_rs = 0.0;
  int last_update_count  = 0;

  // ros::AsyncSpinner spinner(6); // Use 4 threads
  // ros::MultiThreadedSpinner spinner(6);
  // spinner.spin();
  // return 0;

  int last_index        = 0;
  int target_point      = 0;
  int is_during_ref     = 0;
  double cur_lane_width = 0;

  int generate_tip = 0; //默认0 ，平移动后为1。，为1时触发back
  int control_tip  = 0;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();

    // REF初始化规划 当前位置判断
    // if (cur_location.ref_index > 0 && route_ref_line_updata_tip > 0)

    //车辆位置
    rvizPointPub(cur_location.pos_x, cur_location.pos_y);
    // agv蓝色外框
    pushAGVMarker(cur_location.pos_x, cur_location.pos_y, cur_location.heading);

    if (cur_location.ref_index > 0)
    {
      geometry_msgs::Point cur_ref_point;
      cur_ref_point.x         = cur_location.pos_x;
      cur_ref_point.y         = cur_location.pos_y;
      double cur_ref_distance = 0;
      int cur_lane_pos        = 0;
      int ret              = isPointDuingLane(cur_location.ref_index, cur_ref_point, cur_ref_distance, cur_lane_pos, 1);
      cur_location.ref_pos = cur_lane_pos;

      if (cur_lane_pos > 0)
      {
        //重新规划后运动7.5m后重新使能场端定位消息，下一步增加根据距离场端位置决定是否使能
        if (route_ref_line_updata_tip > 0)
        {
          start_loc          = cur_location;
          enable_fixed_lidar = 0;
        }
        else
        {
          // enable_fixed_lidar = 1;
          if (enable_fixed_lidar == 0)
          {
            double drs_temp = rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs -
                              rount_ref_line.REF_line_INFO.at(start_loc.ref_index).rs;
            if (drs_temp > 7.5)
            {
              enable_fixed_lidar = 1;
              ROS_ERROR("Star Pos (%lf,%lf) cur Pos (%lf,%lf) drs %lf fixed tip %d", start_loc.pos_x, start_loc.pos_y,
                        cur_location.pos_x, cur_location.pos_y, drs_temp, enable_fixed_lidar);
            }
            else
            {
              // ROS_WARN("Star Pos (%lf,%lf) cur Pos (%lf,%lf) drs %lf fixed tip %d", start_loc.pos_x,
              // start_loc.pos_y,
              //          cur_location.pos_x, cur_location.pos_y, drs_temp, enable_fixed_lidar);
            }
          }
        }

        //显示车道边界线
        showRefBoundary(cur_location.ref_index, cur_lane_pos);

        double left_boundary_temp =
            rount_ref_line.REF_line_INFO.at(cur_location.ref_index).lane_ranges.at(cur_lane_pos - 1).left_boundary;
        double right_boundary_temp =
            rount_ref_line.REF_line_INFO.at(cur_location.ref_index).lane_ranges.at(cur_lane_pos - 1).right_boundary;
        double lane_width          = 0.0;
        int is_get_new_ref_line    = 0;
        int is_get_fist_right_line = 0;
        int end_point_index        = findEndPointRef(cur_location.ref_index);
        target_point               = end_point_index;
        double max_ds_cur          = mathLanelength(cur_location.ref_index, end_point_index);

        if (left_boundary_temp > 0 && right_boundary_temp > 0)
        { // agv is during right lane
          //判断当前位置到下一个斜行或者转弯区域时长度是否大于 9 + 2×当前点到参考点的距离
          lane_width     = (right_boundary_temp - left_boundary_temp) / 2 + left_boundary_temp;
          cur_lane_width = lane_width;
          //生成右参考直线，相当于已经完成了右斜行，将要直行加左斜行
          if (max_ds_cur > (9 + 2 * abs(lane_width)))
          {
            max_ds_cur             = max_ds_cur - (9 + 2 * abs(lane_width));
            is_get_new_ref_line    = 1;
            is_get_fist_right_line = 1;
            is_during_ref          = 1;
            if (route_ref_line_updata_tip > 0)
            {
              route_ref_line_updata_tip = 0;
              generateFirstNewRefLine(is_get_fist_right_line, cur_location.ref_index, max_ds_cur, end_point_index,
                                      lane_width, left_boundary_temp, right_boundary_temp);
              generate_tip = 1;
            }
          }
        }
        else if (left_boundary_temp < 0 && right_boundary_temp < 0)
        { // agv is during left lane
          //判断当前位置到下一个斜行或者转弯区域时长度是否大于 9 + 2×当前点到参考点的距离
          lane_width     = (left_boundary_temp - right_boundary_temp) / 2 + right_boundary_temp;
          cur_lane_width = lane_width;
          //生成左参考直线，相当于已经完成了左斜行，将要直行加右斜行
          if (max_ds_cur > (9 + 2 * abs(lane_width)))
          {
            max_ds_cur             = max_ds_cur - (9 + 2 * abs(lane_width));
            is_get_new_ref_line    = 1;
            is_get_fist_right_line = -1;
            is_during_ref          = -1;
            if (route_ref_line_updata_tip > 0)
            {
              route_ref_line_updata_tip = 0;
              generateFirstNewRefLine(is_get_fist_right_line, cur_location.ref_index, max_ds_cur, end_point_index,
                                      lane_width, left_boundary_temp, right_boundary_temp);
              generate_tip = 1;
            }
          }
        }
        else
        { // agv is during ref lane
          is_during_ref = 0;
          if (route_ref_line_updata_tip > 0)
          {
            route_ref_line_updata_tip = 0;
            generateRefLine(cur_location.ref_index, end_point_index);
          }
        }
        // ROS_INFO("Cur (%lf,%lf) -> ref %d (%lf,%lf) ds %lf", cur_location.pos_x, cur_location.pos_y,
        //          cur_location.ref_index, rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rx,
        //          rount_ref_line.REF_line_INFO.at(cur_location.ref_index).ry, max_ds_cur);
      }
      else
      {
        // ROS_ERROR("The agv pos is %d", cur_lane_pos);
      }
    }
    else
    {
      // ROS_INFO("Wait location vec OK");
    }

    //激光检测
    if (lidar_detection_recieve_tip == 1)
    {
      lidar_detection_recieve_tip = 0;

      int count_lidar_vec = static_cast< int >(lidar_detection_obs_vec.size());
      if (count_lidar_vec > 0)
      {
        detectionLidarObsJudge();
        obstacle_detection_judge_tip = 0;
        lidar_detection_obs_vec.clear();
      }
      control_tip = 0;
    }

    //相机检测
    if (cam_detection_recieve_tip == 1)
    {
      cam_detection_recieve_tip = 0;
      int count_camera_vec      = static_cast< int >(camera_detection_obs_vec.size());
      if (count_camera_vec > 0)
      {
        // detectionCameraObsJudge(cur_location, camera_detection_obs_vec);
        detectionCameraObsJudge();
        // obstacle_detection_judge_tip = 0;
        camera_detection_obs_vec.clear();
      }
    }

    //障碍物检测分类
    // 0、障碍物对应的地图位置
    // 1、静态障碍无动态障碍物区分；
    // 2、障碍物大小区分，小、中、大
    // 3、障碍物追踪
    // 4、障碍物行为预测
    // 5、障碍物行为结果对行驶车道的影响
    // 6、agv行为预测
    // 7、车道生成规则
    // 8、障碍物四个角点相似度

    // controlAgvPub(1.5, 0);

    int count_vec = static_cast< int >(obstacle_detection_vec.size());
    if (count_vec > 0)
    {
      ROS_INFO("obs judge -------%d-------", count_vec);
      // ROS_ERROR("Find %d obs during", count);
      //判断障碍物距离
      double min_obs_rs  = 45.0;
      double ref_rs_temp = 45.0;
      // double &cur_rs     = rount_ref_line.REF_line_INFO.at(cur_location.ref_index).rs;

      for (int i = 0; i < count_vec; i++)
      {
        min_obs_rs = min(min_obs_rs, obstacle_detection_vec.at(i).min_y);
      }
      ROS_INFO("Find min obs distance is %lf", min_obs_rs);

      if (min_obs_rs < (20.0 + 9.0) && min_obs_rs >= (15.0 + 9.0))
      {
        ROS_ERROR("slow speed-------");
        control_tip = 1;
        // controlAgvPub(0.8, 0);
      }
      else if (min_obs_rs < (15.0 + 9.0) && min_obs_rs >= (10.0 + 9.0))
      {
        // controlAgvPub(0.5, 0);
        control_tip = 2;
        //回参考线
        if (is_during_ref == 1 && generate_tip == 1)
        {
          generate_tip = 0;
          generateBackRefLine(is_during_ref, cur_location.ref_index, target_point, cur_lane_width);
          ROS_ERROR("right back ref-------");
        }
        else if (is_during_ref == -1 && generate_tip == 1)
        {
          generate_tip = 0;
          generateBackRefLine(is_during_ref, cur_location.ref_index, target_point, cur_lane_width);
          ROS_ERROR("left back ref-------");
        }
        else
        {
          ROS_ERROR("on ref-------");
        }
        //不处理
      }
      else if (min_obs_rs <= (10.0 + 9.0) && min_obs_rs > (8.0 + 9.0))
      {
        ROS_ERROR("slow speed-------");
        control_tip = 2;
      }
      else if (min_obs_rs <= (8.0 + 9.0) && min_obs_rs > (5.0 + 9.0))
      {
        ROS_ERROR("stop-------");
        control_tip = 3;
        // controlAgvPub(0, 0);
      }
      else if (min_obs_rs <= (5.0 + 9.0))
      {
        ROS_ERROR("E stop-------");
        control_tip = 4;
        // controlAgvPub(0, 1);
      }
      else
      {
        control_tip = 0;
      }
    }

    switch (control_tip)
    {
    case 0:
      controlAgvPub(1.5, 0);
      break;
    case 1:
      controlAgvPub(0.8, 0);
      ROS_ERROR("slow speed -- 0.8 m/s -- cur v %lf %lf", agv_status_info.ActualSpd, cur_location.total_v);
      break;
    case 2:
      controlAgvPub(0.5, 0);
      ROS_ERROR("slow speed -- 0.5 m/s -- cur v %lf %lf", agv_status_info.ActualSpd, cur_location.total_v);
      break;
    case 3:
      controlAgvPub(0, 0);
      ROS_ERROR("stop speed -- 0.0 m/s -- cur v %lf %lf", agv_status_info.ActualSpd, cur_location.total_v);
      break;
    case 4:
      if (recv_agv_status_tip == 1)
      {
        if (abs(agv_status_info.ActualSpd) < 0.1 && cur_location.total_v < 0.1)
        {
          controlAgvPub(0, 0);
          ROS_ERROR("stop replace E_stop -- speed 0.0 m/s -- cur v %lf %lf", agv_status_info.ActualSpd,
                    cur_location.total_v);
        }
        else
        {
          controlAgvPub(0, 1);
          ROS_ERROR("E_stop -- speed 0.0 m/s -- cur v %lf %lf", agv_status_info.ActualSpd, cur_location.total_v);
        }
      }
      else
      {
        if (cur_location.total_v < 0.1)
        {
          controlAgvPub(0, 0);
          ROS_ERROR("stop replace E_stop -- speed 0.0 m/s -- cur v %lf %lf -- without agvstatus",
                    agv_status_info.ActualSpd, cur_location.total_v);
        }
        else
        {
          controlAgvPub(0, 1);
          ROS_ERROR("E_stop -- speed 0.0 m/s -- cur v %lf %lf -- without agvstatus", agv_status_info.ActualSpd,
                    cur_location.total_v);
        }
      }
      break;
    default:
      controlAgvPub(1.5, 0);
    }

    loop_rate.sleep();
    ++count;
  }

  // ROS_INFO("shutting down!\n");
  // ros::shutdown();

  return 0;
}
