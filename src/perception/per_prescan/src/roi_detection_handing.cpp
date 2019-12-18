#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <sstream>
#include <string>
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

#include "glog_helper.h"

#include "get_ref_map.h"
#include "math_interpolation.h"
#include "roi_around_agv.h"

using namespace std;
using namespace superg_agv::roi_agv;
using namespace superg_agv::roi_math;

using std::__cxx11::string;

#define LOCA_MAX_SIZE 10 * 10

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

struct TaskPoint
{
  double x;
  double y;
  double heading;
  double max_speed;
  int laneID;
  int task_click_recv_tip;
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

// ros::NodeHandle n;
// //计算线性插值 B
MathInterData *math_inter_data;
//获取地图大小 B
GetRefMapAreaRange *get_map_range;
//制作ROI区域
BuildRoiSpace *new_roi;

//
ros::Subscriber fusion_location_sub;  //融合定位
ros::Subscriber fusion_obstacle_sub;  //融合感知结果
ros::Subscriber cam_sub;              //相机感知
ros::Subscriber lidar_sub;            //激光感知
ros::Subscriber ref_line_sub;         //参考线
ros::Subscriber task_click_sub;       //点击点
ros::Subscriber lidar_obu_sub;        // obu
ros::Subscriber agv_status_sub;       // agv status

LocationTemp last_location;
LocationTemp cur_location;
LocationTemp lidar_location_obs_time;
TaskPoint task_click;
std::vector<LocationTemp> location_vec;
map_msgs::REFPointArray rount_ref_line;
control_msgs::AGVStatus agv_status_info;
std::vector<common_msgs::DetectionInfo> lidar_detection_obs_vec;
std::vector<Point> lane_boundry_points;
std::vector<Point> lane_left_boundry_points;
std::vector<Point> lane_right_boundry_points;

int route_ref_line_updata_tip = 0;  //初始化为0,收到新的为1,发送到车后为0
int fusion_location_updata_tip = 0;
int recv_agv_status_tip = 0;
int location_index = 0;
int location_vec_tip = 0;
int lidar_detection_recieve_tip = 0;
double map_scale = 1;  //地图放大缩小等级
//
Point base_point;
Point max_point;

double mathPointDistanceSquare(double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

int findAgvLocation(double x_, double y_, double heading_, int last_pos_index, int isFastJudge, int isHeadingJudge)
{
  double last_distance = -1.0;
  double cur_distance = -1.0;
  double next_distance = -1.0;
  double min_delta_heading = 360.0;
  double delta_heading = 0.0;
  double min_distance = 100;
  int min_index = 0;
  int begin_index = 3;

  if (!rount_ref_line.REF_line_INFO.empty())
  {
    int count = static_cast<int>(rount_ref_line.REF_line_INFO.size());

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
        delta_heading = abs(rount_ref_line.REF_line_INFO.at(i - 1).rtheta - heading_);
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
            min_index = i - 1;
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
                min_index = i - 1;
              }
            }
          }
          else
          {
            if (min_distance > cur_distance)
            {
              min_distance = cur_distance;
              min_index = i - 1;
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

Point checkMaskRange(Point &p_)
{
  Point p_temp;
  if (p_.x < 0)
  {
    p_temp.x = 0;
  }
  else
  {
    if (p_.x >= max_point.x)
    {
      p_temp.x = max_point.x - 1;
    }
    else
    {
      p_temp.x = p_.x;
    }
  }

  if (p_.y < 0)
  {
    p_temp.y = 0;
  }
  else
  {
    if (p_.y >= max_point.y)
    {
      p_temp.y = max_point.y - 1;
    }
    else
    {
      p_temp.y = p_.y;
    }
  }
  return p_temp;
}

void recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
  location_index++;
  cur_location.time = location_msg->header.stamp;
  cur_location.index = location_index;
  cur_location.pos_x = location_msg->pose.x;
  cur_location.pos_y = location_msg->pose.y;

  // cur_location.pos_x = -190.029;
  // cur_location.pos_y = -45.2474;

  cur_location.heading = location_msg->yaw;
  cur_location.speed_x = location_msg->velocity.linear.x;
  cur_location.speed_y = location_msg->velocity.linear.y;
  cur_location.total_v = abs(cur_location.speed_x) + abs(cur_location.speed_y);  // 1m/s
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
    location_vec_tip = 1;
    std::vector<LocationTemp>::iterator k = location_vec.begin();
    location_vec.erase(k);  //删除第一个元素
  }
  cur_location.ref_index =
      findAgvLocation(cur_location.pos_x, cur_location.pos_y, cur_location.heading, cur_location.ref_index, 0, 0);
  fusion_location_updata_tip = 1;
}

void recvFusionObuCallback(const perception_msgs::FusionDataInfo::ConstPtr &obu_msg)
{
}

void recvCamObsCallback(const perception_sensor_msgs::ObjectList::ConstPtr &msg)
{
}

void setLaneContours()
{
  lane_boundry_points.clear();
  lane_boundry_points = lane_left_boundry_points;
  lane_boundry_points.insert(lane_boundry_points.end(), lane_right_boundry_points.rbegin(),
                             lane_right_boundry_points.rend());

  lane_left_boundry_points.clear();
  lane_right_boundry_points.clear();

  // std::cout << "lane_boundry_points is: " << std::endl;
  // std::cout << lane_boundry_points << std::endl;

  ROS_WARN("Left  B:(%d,%d) Left  E:(%d,%d)", lane_left_boundry_points.begin()->x, lane_left_boundry_points.begin()->y,
           lane_left_boundry_points.rbegin()->x, lane_left_boundry_points.rbegin()->y);

  ROS_WARN("Right B:(%d,%d) Right E:(%d,%d)", lane_right_boundry_points.begin()->x,
           lane_right_boundry_points.begin()->y, lane_right_boundry_points.rbegin()->x,
           lane_right_boundry_points.rbegin()->y);

  ROS_WARN("Lane  B:(%d,%d) Lane  E:(%d,%d)", lane_boundry_points.begin()->x, lane_boundry_points.begin()->y,
           lane_boundry_points.rbegin()->x, lane_boundry_points.rbegin()->y);

  new_roi->setContours(lane_boundry_points, 0);
}

void recvLidarObsCallback(const perception_sensor_msgs::ObjectList::ConstPtr &msg)
{
  int obs_num = (int)(msg->obstacle_num);
  if (obs_num > 0)
  {
    lidar_detection_obs_vec.clear();
    lidar_location_obs_time.time = msg->header.stamp;
    lidar_detection_obs_vec = msg->object_list;
    ROS_INFO("recv lidar obs %d %lf", obs_num, lidar_location_obs_time.time.toSec());
  }
  lidar_detection_recieve_tip = 1;
}

void recvRefCallback(const map_msgs::REFPointArray::ConstPtr &msg)
{
  rount_ref_line.header.stamp = msg->header.stamp;

  rount_ref_line.target_lane_ID = msg->target_lane_ID;
  rount_ref_line.agv_lane_ID = msg->agv_lane_ID;

  rount_ref_line.REF_line_INFO.clear();

  int count_line = static_cast<int>(msg->REF_line_INFO.size());

  lane_left_boundry_points.clear();
  lane_right_boundry_points.clear();

  double last_radian_ = 0;
  int last_lane_id = -1;

  Point last_p_temp_left;
  Point last_p_temp_right;

  new_roi->clearLaneContour();

  for (int i = 0; i < count_line; i++)
  {
    common_msgs::REFPoint ref_point_temp;
    ref_point_temp.rs = msg->REF_line_INFO.at(i).rs;
    ref_point_temp.rx = msg->REF_line_INFO.at(i).rx;
    ref_point_temp.ry = msg->REF_line_INFO.at(i).ry;
    ref_point_temp.rtheta = msg->REF_line_INFO.at(i).rtheta;
    ref_point_temp.max_speed = msg->REF_line_INFO.at(i).max_speed;
    ref_point_temp.rkappa = msg->REF_line_INFO.at(i).rkappa;
    ref_point_temp.rdkappa = msg->REF_line_INFO.at(i).rdkappa;
    ref_point_temp.max_speed = msg->REF_line_INFO.at(i).max_speed;
    ref_point_temp.lane_id = msg->REF_line_INFO.at(i).lane_id;
    ref_point_temp.line_direction = msg->REF_line_INFO.at(i).line_direction;

    //地图格式左负右正,算法对应修改2019.10.10
    double temp_left = 0;   //正
    double temp_right = 0;  //负

    int count_boundary = static_cast<int>(msg->REF_line_INFO.at(i).lane_ranges.size());
    for (int j = 0; j < count_boundary; j++)
    {
      common_msgs::LaneRange lane_range_;
      lane_range_.left_boundary = msg->REF_line_INFO.at(i).lane_ranges.at(j).left_boundary;
      lane_range_.right_boundary = msg->REF_line_INFO.at(i).lane_ranges.at(j).right_boundary;
      if (j == 0)
      {
        temp_left = lane_range_.left_boundary;
        temp_right = lane_range_.right_boundary;
      }
      else
      {
        temp_left = min(temp_left, lane_range_.left_boundary);
        temp_right = max(temp_right, lane_range_.right_boundary);
      }
      ref_point_temp.lane_ranges.push_back(lane_range_);
    }

    double radian_ = (ref_point_temp.rtheta) * M_PI / 180;

    //最后一次角度初始化
    if (i == 0)
    {
      last_radian_ = radian_;
      last_lane_id = ref_point_temp.lane_id;
    }

    if (ref_point_temp.line_direction == 8 || ref_point_temp.line_direction == 16)
    {
      radian_ = last_radian_;
    }

    double left_x = map_scale * (ref_point_temp.rx + temp_left * cos(radian_));
    double left_y = map_scale * (ref_point_temp.ry - temp_left * sin(radian_));
    double right_x = map_scale * (ref_point_temp.rx + temp_right * cos(radian_));
    double right_y = map_scale * (ref_point_temp.ry - temp_right * sin(radian_));

    Point p_temp_left;
    p_temp_left.x = new_roi->getIntFromDouble(left_x) - base_point.x;
    p_temp_left.y = new_roi->getIntFromDouble(left_y) - base_point.y;
    p_temp_left = checkMaskRange(p_temp_left);
    lane_left_boundry_points.push_back(p_temp_left);
    // ROS_INFO("left (%d,%d)", p_temp.x, p_temp.y);
    Point p_temp_right;
    p_temp_right.x = new_roi->getIntFromDouble(right_x) - base_point.x;
    p_temp_right.y = new_roi->getIntFromDouble(right_y) - base_point.y;
    p_temp_right = checkMaskRange(p_temp_right);
    lane_right_boundry_points.push_back(p_temp_right);
    // ROS_INFO("right (%d,%d)", p_temp.x, p_temp.y);
    //分路段填值

    // if (i == 0)
    // {
    //   last_p_temp_left  = p_temp_left;
    //   last_p_temp_right = p_temp_right;
    // }

    if (last_lane_id != ref_point_temp.lane_id)
    {
      setLaneContours();

      p_temp_left.x = new_roi->getIntFromDouble(left_x) - base_point.x;
      p_temp_left.y = new_roi->getIntFromDouble(left_y) - base_point.y;
      p_temp_left = checkMaskRange(p_temp_left);
      lane_left_boundry_points.push_back(p_temp_left);
      p_temp_right.x = new_roi->getIntFromDouble(right_x) - base_point.x;
      p_temp_right.y = new_roi->getIntFromDouble(right_y) - base_point.y;
      p_temp_right = checkMaskRange(p_temp_right);
      lane_right_boundry_points.push_back(p_temp_right);
    }

    last_radian_ = radian_;
    last_lane_id = ref_point_temp.lane_id;

    // last_p_temp_left  = p_temp_left;
    // last_p_temp_right = p_temp_right;

    rount_ref_line.REF_line_INFO.push_back(ref_point_temp);

    ROS_WARN("REF id:%d (%lf,%lf) theta:%lf ,left (%lf,%lf) ,right (%lf,%lf) ", ref_point_temp.lane_id,
             ref_point_temp.rx, ref_point_temp.ry, radian_, p_temp_left.x, p_temp_left.y, p_temp_right.x,
             p_temp_right.y);
  }

  setLaneContours();

  route_ref_line_updata_tip = 1;

  ROS_INFO("updata ref line lane %u -> %u total %d point %lf m", rount_ref_line.agv_lane_ID,
           rount_ref_line.target_lane_ID, static_cast<int>(rount_ref_line.REF_line_INFO.size()),
           rount_ref_line.REF_line_INFO.rbegin()->rs);
}

void recvClickCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  task_click.x = msg->point.x;
  task_click.y = msg->point.y;
  task_click.laneID = (int)msg->point.z;
  task_click.task_click_recv_tip = 1;
  ROS_INFO("rcev rviz click: (%lf,%lf) in lane %d", task_click.x, task_click.y, task_click.laneID);
}

void agvStatusCallback(const control_msgs::AGVStatus::ConstPtr &agv_status_msg)
{
  agv_status_info.ActualSpd = agv_status_msg->ActualSpd;
  recv_agv_status_tip = 1;
}

void doAgvTransformationToLocal(double temp_x_, double temp_y_, double x_, double y_, double radian_, Point &p)
{
  p.x = x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_);
  p.y = y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_);
  p.x = new_roi->getIntFromDouble(p.x) - base_point.x;
  p.y = new_roi->getIntFromDouble(p.y) - base_point.y;
  p = checkMaskRange(p);
}

void doAgvTransformationToLocal(double temp_x_, double temp_y_, double x_, double y_, double radian_, double map_size,
                                Point &p)
{
  p.x = (x_ + temp_y_ * sin(radian_) - temp_x_ * cos(radian_)) * map_size;
  p.y = (y_ + temp_y_ * cos(radian_) + temp_x_ * sin(radian_)) * map_size;
  p.x = new_roi->getIntFromDouble(p.x) - base_point.x;
  p.y = new_roi->getIntFromDouble(p.y) - base_point.y;
  p = checkMaskRange(p);
}

void obsRange(double min_x_, double max_x_, double min_y_, double max_y_, double map_size,
              std::vector<Point> &point_vec_)
{
  double base_location_x = cur_location.pos_x * map_size;
  double base_location_y = cur_location.pos_y * map_size;
  double radian_ = cur_location.heading * M_PI / 180;

  point_vec_.clear();

  Point p_temp;
  double temp_x_ = min_x_ * map_size;
  double temp_y_ = min_y_ * map_size;
  doAgvTransformationToLocal(temp_x_, temp_y_, base_location_x, base_location_y, radian_, p_temp);
  point_vec_.push_back(p_temp);

  temp_x_ = min_x_ * map_size;
  temp_y_ = max_y_ * map_size;
  doAgvTransformationToLocal(temp_x_, temp_y_, base_location_x, base_location_y, radian_, p_temp);
  point_vec_.push_back(p_temp);

  temp_x_ = max_x_ * map_size;
  temp_y_ = max_y_ * map_size;
  doAgvTransformationToLocal(temp_x_, temp_y_, base_location_x, base_location_y, radian_, p_temp);
  point_vec_.push_back(p_temp);

  temp_x_ = max_x_ * map_size;
  temp_y_ = min_y_ * map_size;
  doAgvTransformationToLocal(temp_x_, temp_y_, base_location_x, base_location_y, radian_, p_temp);
  point_vec_.push_back(p_temp);
}

void locationRange(double front_, double back_, double left_, double right_, double map_size,
                   std::vector<Point> &point_vec_)
{
  double base_location_x = cur_location.pos_x * map_size;
  double base_location_y = cur_location.pos_y * map_size;
  double radian_ = cur_location.heading * M_PI / 180;

  front_ = front_ * map_size;
  back_ = back_ * map_size;
  left_ = left_ * map_size;
  right_ = right_ * map_size;

  point_vec_.clear();

  geometry_msgs::Point p;
  Point p_temp;
  p.z = 0;

  p.x = base_location_x + front_ * sin(radian_) - left_ * cos(radian_);
  p.y = base_location_y + front_ * cos(radian_) + left_ * sin(radian_);
  p_temp.x = new_roi->getIntFromDouble(p.x) - base_point.x;
  p_temp.y = new_roi->getIntFromDouble(p.y) - base_point.y;
  p_temp = checkMaskRange(p_temp);
  point_vec_.push_back(p_temp);

  p.x = base_location_x + front_ * sin(radian_) + right_ * cos(radian_);
  p.y = base_location_y + front_ * cos(radian_) - right_ * sin(radian_);
  p_temp.x = new_roi->getIntFromDouble(p.x) - base_point.x;
  p_temp.y = new_roi->getIntFromDouble(p.y) - base_point.y;
  p_temp = checkMaskRange(p_temp);
  point_vec_.push_back(p_temp);

  p.x = base_location_x - back_ * sin(radian_) + right_ / 2 * cos(radian_);
  p.y = base_location_y - back_ * cos(radian_) - right_ / 2 * sin(radian_);
  p_temp.x = new_roi->getIntFromDouble(p.x) - base_point.x;
  p_temp.y = new_roi->getIntFromDouble(p.y) - base_point.y;
  p_temp = checkMaskRange(p_temp);
  point_vec_.push_back(p_temp);

  p.x = base_location_x - back_ * sin(radian_) - left_ * cos(radian_);
  p.y = base_location_y - back_ * cos(radian_) + left_ * sin(radian_);
  p_temp.x = new_roi->getIntFromDouble(p.x) - base_point.x;
  p_temp.y = new_roi->getIntFromDouble(p.y) - base_point.y;
  p_temp = checkMaskRange(p_temp);
  point_vec_.push_back(p_temp);
}

void detectionLidarObsJudge(double map_size)
{
  // int ans = getLocationFromVectorTemp(lidar_location_obs_time);
  int ans = 1;
  if (ans == 1)
  {
    //障碍物判断初值
    int id_index = 0;
    // double radian_ = lidar_location_obs_time.heading * M_PI / 180;
    // double x_      = lidar_location_obs_time.pos_x* map_size;
    // double y_      = lidar_location_obs_time.pos_y* map_size;
    //临时使用实时定位数据0925
    double radian_ = cur_location.heading * M_PI / 180;
    double x_ = cur_location.pos_x;
    double y_ = cur_location.pos_y;
    double temp_x_ = 0;
    double temp_y_ = 0;
    //更新查询参考线位置
    int ref_index = cur_location.ref_index;
    if (ref_index > 0 && ref_index < (static_cast<int>(rount_ref_line.REF_line_INFO.size()) - 2))
    {
      int obs_num = static_cast<int>(lidar_detection_obs_vec.size());
      std::vector<common_msgs::DetectionInfo>::iterator lidar_it;
      new_roi->initObsMap();
      new_roi->clearContour();
      for (lidar_it = lidar_detection_obs_vec.begin(); lidar_it != lidar_detection_obs_vec.end(); lidar_it++)
      {
        ObstacleDetectionTemp obstacle_temp;

        obstacle_temp.ref_index = ref_index;
        lidar_it->state[0];
        obstacle_temp.obs_id = lidar_it->id;
        obstacle_temp.max_x = (0 - lidar_it->state[0]);
        obstacle_temp.min_y = lidar_it->state[1];
        obstacle_temp.min_x = (0 - lidar_it->state[2]);
        obstacle_temp.max_y = lidar_it->state[3];

        double judge_y_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));
        double judge_x_temp = min(abs(obstacle_temp.min_y), abs(obstacle_temp.max_y));

        int obs_math_tip = 0;

        if (judge_y_temp < 40.0)
        {
          obs_math_tip = 1;

          std::vector<Point> point_vec_;
          Point p_temp;

          // ROS_INFO("obstacle_temp min(%lf,%lf), max(%lf,%lf), loca(%lf,%lf)", obstacle_temp.min_x,
          // obstacle_temp.min_y,
          //          obstacle_temp.max_x, obstacle_temp.max_y, x_, y_);

          id_index++;
          visualization_msgs::Marker points;
          //转换全局
          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, map_size, p_temp);
          point_vec_.push_back(p_temp);
          // std::cout << "p_temp is: \n" << p_temp << std::endl;

          temp_x_ = obstacle_temp.min_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, map_size, p_temp);
          point_vec_.push_back(p_temp);
          // std::cout << "p_temp is: \n" << p_temp << std::endl;

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.max_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, map_size, p_temp);
          point_vec_.push_back(p_temp);
          // std::cout << "p_temp is: \n" << p_temp << std::endl;

          temp_x_ = obstacle_temp.max_x;
          temp_y_ = obstacle_temp.min_y;
          doAgvTransformationToLocal(temp_x_, temp_y_, x_, y_, radian_, map_size, p_temp);
          point_vec_.push_back(p_temp);
          // std::cout << "p_temp is: \n" << p_temp << std::endl;
          new_roi->setContours(point_vec_, 3);
        }
        if (obs_math_tip == 1)
        {
          new_roi->useObsMask(3);
        }
      }
      // ROS_INFO("mathObsAndROI");
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "roi_obs");
  ros::NodeHandle n;

  //设置参数
  GLogHelper gh(argv[0]);

  //计算线性插值 B
  math_inter_data = new MathInterData();

  std::vector<InterData3D> data_in;
  std::vector<InterData3D> data_out;
  int total_inter_num = 100;
  int inter_method = 1;

  InterData3D data_temp;
  ros::Time time_temp = ros::Time::now();
  data_temp.data_x = 70.7;
  data_temp.data_y = 30.5;
  data_temp.time = time_temp.toSec();
  data_in.push_back(data_temp);
  sleep(1);

  time_temp = ros::Time::now();
  data_temp.data_x = 60.7;
  data_temp.data_y = 40.5;
  data_temp.time = time_temp.toSec();
  data_in.push_back(data_temp);
  sleep(1);

  time_temp = ros::Time::now();
  data_temp.data_x = 50.7;
  data_temp.data_y = 50.5;
  data_temp.time = time_temp.toSec();
  data_in.push_back(data_temp);
  sleep(1);

  ros::Time b_time = ros::Time::now();
  math_inter_data->mathInterpolation3D(data_in, data_out, total_inter_num, inter_method);
  ros::Time e_time = ros::Time::now();
  // for (size_t i = 0; i < total_inter_num; i++)
  // {
  //   ROS_INFO("Out %d: %lf %lf %lf", i, data_out.at(i).data_x, data_out.at(i).data_y, data_out.at(i).time);
  // }
  // ROS_WARN("Inter time: %lf", e_time.toSec() - b_time.toSec());
  //计算线性插值 B

  //获取地图大小 B
  get_map_range = new GetRefMapAreaRange();

  std::string workplace_path = "/work/superg_agv/src/routing/map_new/data";
  std::string file_name = "/area.json";
  AreaRange map_range;
  get_map_range->setAreaFilePath(workplace_path, file_name);
  get_map_range->getAreaRangeFromFile(map_range);
  ROS_INFO("double min x %lf max x %lf min y %lf max y %lf", map_range.min_x, map_range.max_x, map_range.min_y,
           map_range.max_y);
  //获取地图大小 E

  //制作ROI区域
  new_roi = new BuildRoiSpace(n);
  // new_roi = new BuildRoiSpace();

  map_scale = 4;                    //地图放大缩小等级
  new_roi->setMapScale(map_scale);  //原始单位为m
  new_roi->setMaxAndMinWidthHeight(map_range.max_x, map_range.min_x, map_range.max_y, map_range.min_y);
  new_roi->setMapSize();
  new_roi->initMap();
  new_roi->showMaxMinWidthHeight();

  base_point = new_roi->getBaseXY();
  max_point = new_roi->getMaxXY();
  ROS_WARN("base point (%d,%d) max (%d,%d)", base_point.x, base_point.y, max_point.x, max_point.y);

  fusion_location_sub = n.subscribe("/localization/fusion_msg", 5, recvFusionLocationCallback);
  fusion_obstacle_sub = n.subscribe("/perception/obstacle_info", 5, recvFusionObuCallback);
  cam_sub = n.subscribe("/drivers/perception/camera_obstacle_info", 5, recvCamObsCallback);
  lidar_sub = n.subscribe("/perception/detection_lidar", 5, recvLidarObsCallback);
  // lidar_sub      = n.subscribe("/perception/detection_lidar_test", 5, recvLidarObsCallback);
  ref_line_sub = n.subscribe("/map/ref_point_info", 5, recvRefCallback);
  task_click_sub = n.subscribe("/monitor/rviz_click_lane", 5, recvClickCallback);
  agv_status_sub = n.subscribe("/drivers/com2agv/agv_status", 5, agvStatusCallback);

  int count = 0;

  double last_min_obs_rs = 0.0;
  int last_update_count = 0;

  // ros::AsyncSpinner spinner(6); // Use 4 threads
  // ros::MultiThreadedSpinner spinner(6);
  // spinner.spin();
  // return 0;

  int last_index = 0;
  int target_point = 0;
  int is_during_ref = 0;
  double cur_lane_width = 0;

  int generate_tip = 0;  //默认0 ，平移动后为1。，为1时触发back
  int control_tip = 0;

  last_location.pos_x = 0;
  last_location.pos_y = 0;
  last_location.heading = 0;

  double max_d_time = -1;
  double min_d_time = 1000;
  double ave_time = 0;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();

    if (fusion_location_updata_tip == 1)
    {
      fusion_location_updata_tip = 0;

      //激光检测
      if (lidar_detection_recieve_tip == 1)
      {
        lidar_detection_recieve_tip = 0;

        int count_lidar_vec = static_cast<int>(lidar_detection_obs_vec.size());
        if (count_lidar_vec > 0)
        {
          b_time = ros::Time::now();
          detectionLidarObsJudge(map_scale);
          e_time = ros::Time::now();
          double d_time = e_time.toSec() - b_time.toSec();
          max_d_time = max(max_d_time, d_time);
          min_d_time = min(min_d_time, d_time);
          ROS_WARN("Roi map time: %lf  max:%lf  min:%lf", d_time, max_d_time, min_d_time);
          lidar_detection_obs_vec.clear();
        }
        // control_tip = 0;
      }

      double distance_temp =
          mathPointDistanceSquare(cur_location.pos_x, cur_location.pos_y, last_location.pos_x, last_location.pos_y);
      double angle_temp = abs(cur_location.heading - last_location.heading);
      if (distance_temp > 3 || angle_temp > 5)
      {
        // new_roi->initMaskMap();
        new_roi->clearAgvContour();

        // std::vector< Point > agv_boundry_points;
        // locationRange(8.0, 8.0, 1.5, 1.5, map_scale, agv_boundry_points);
        // new_roi->setContours(agv_boundry_points, 1);

        std::vector<Point> location_boundry_points;
        locationRange(50, 30, 15, 10, map_scale, location_boundry_points);
        // new_roi->setContours(location_boundry_points, 2);
        new_roi->refreshROIMap();

        // std::cout << "location_boundry_points is: " << std::endl;
        // std::cout << location_boundry_points << std::endl;
        // new_roi->initObsMap();
        // new_roi->clearContour();
        std::vector<Point> obs_points;
        locationRange(8, 8, 1.5, 1.5, map_scale, obs_points);
        // // new_roi->clearContour();
        // new_roi->setContours(obs_points, 3);
        // obsRange(-9, -8, 20, 22, map_scale, obs_points);
        // // new_roi->clearContour();
        // new_roi->setContours(obs_points, 3);
        // obsRange(8, 9, -22, 100, map_scale, obs_points);
        // // new_roi->clearContour();
        new_roi->setContours(obs_points, 3);
        new_roi->useObsMask(3);
        // new_roi->mathObsAndROI();

        new_roi->mathObsAndROI();
        new_roi->showAllMap(0);
        last_location = cur_location;
      }

      if (route_ref_line_updata_tip == 1)
      {
        route_ref_line_updata_tip = 0;

        // std::vector< Point > lane_right_boundry_points_rever;
        // lane_right_boundry_points_rever.reserve(lane_right_boundry_points.size());

        new_roi->refreshROIMap();

        // new_roi->showAllMap(0);
      }

      // new_roi->coutROIMap();
    }

    loop_rate.sleep();
    ++count;
  }

  // ROS_INFO("shutting down!\n");
  // ros::shutdown();

  return 0;
}
