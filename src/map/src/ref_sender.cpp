#include "ref_sender.h"

#define TESTMODE1 0  //局部路径发布测试 //寻找当前点+道路id
#define TESTMODE2 0  //判断任务结束，重启任务

using namespace std;

namespace superg_agv
{
namespace map
{
RefSender::RefSender(ros::NodeHandle &nh, const double &point_x_, const double &point_y_, const int &para_tip_)
  : nh_(nh)
{
  ROS_INFO("RefSender establish!");

  refSenderInit();

  if (para_tip_ == 3)
  {
    vcu_location_recv_tip = 1;
    vcu_location_.x = point_x_;  //经  x
    vcu_location_.y = point_y_;  //纬  y
  }

  location_fusion_sub_ = nh.subscribe("/localization/fusion_msg", 10, &RefSender::recvFusionLocationCallback, this);
  task_click_sub_ = nh.subscribe("/monitor/rviz_click_lane", 10, &RefSender::recvClickCallback, this);
  vcu_locatio_sub_ = nh.subscribe("/localization/fusion_msg", 10, &RefSender::recvVCUCallback, this);
  route_laneID_array_sub_ = nh.subscribe("/map/route_laneID_arry", 10, &RefSender::recvRouteLaneIDArrayCallback, this);
  hmi_control_sub_ = nh.subscribe("/monitor/hmi_control_ad", 10, &RefSender::recvHMIControlADCallback, this);
  ultra_info_sub_ = nh.subscribe("/drivers/can_wr/sonser_info", 10, &RefSender::recvUltraInfoCallback, this);
  ad_status_sub_ = nh.subscribe("/plan/ad_status", 10, &RefSender::recvADStatusCallback, this);

  vms_info_pub_ = nh.advertise<hmi_msgs::VMSControlAD>("/map/vms_control_info", 10, true);
  route_ref_planning_pub_ = nh.advertise<map_msgs::REFPointArray>("/map/ref_point_info", 10, true);
  vcu_location_pub_ = nh.advertise<geometry_msgs::PointStamped>("/map/vcu_location_lane", 10, true);
  rviz_all_point_pub_ = nh.advertise<visualization_msgs::Marker>("/map/all_map_msg", 1, true);
  rviz_route_ref_pub_ = nh.advertise<visualization_msgs::Marker>("/map/route_msg", 1, true);
  rviz_bj_point_pub_ = nh.advertise<visualization_msgs::Marker>("/map/all_bj_line_msg", 1, true);
  rviz_vcu_point_pub_ = nh.advertise<visualization_msgs::Marker>("/map/all_vcu_line_msg", 1, true);
  rviz_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/map/veh_point_msg", BUF_LEN, true);
  rviz_color_test_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/map/color_test", 1, true);
  route_decision_info_pub_ = nh.advertise<plan_msgs::DecisionInfo>("/map/decision_info", 10, true);
  rviz_appendix_attributemap_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/map/aa_msg", 1, true);
  // route_decision_info_pub_ = nh.advertise< plan_msgs::DecisionInfo >("/plan/decision_info", 10, true);

  rvizColorTest();

  get_ref_line = new GetRefLine();
  readRefFromGetRefLine();
  readRefFromCSV();
  refSender();
}

void RefSender::readRefFromGetRefLine()
{
  std::string home_path = getenv("HOME");
  std::string workplace_path = "/work/superg_agv/src/routing/map_new/data";
  std::string ref_line_file_name = home_path + workplace_path + "/ref_line.json";
  std::string ref_line_appendix_file_name = home_path + workplace_path + "/appendix_belongs.json";
  std::string ref_point_file_name = home_path + workplace_path + "/ref_points.json";
  std::string appendix_attribute_name = home_path + workplace_path + "/appendix_attribute.json";
  std::string bj_line_file_name = home_path + workplace_path + "/bj_line.json";
  std::string bj_point_file_name = home_path + workplace_path + "/bj_points.json";
  std::string pysical_circle_name = home_path + workplace_path + "/physical_circle.json";
  std::string pysical_line_name = home_path + workplace_path + "/physical_line.json";
  std::string pysical_points_name = home_path + workplace_path + "/physical_points.json";

  get_ref_line->setRefLineInit();
  get_ref_line->getRefLineFromFile2Map(ref_line_file_name, ref_line_map);
  get_ref_line->getAppendixFromFile2RefLine(ref_line_appendix_file_name, ref_line_map);
  get_ref_line->getRefPointFromFile2Map(ref_point_file_name, ref_line_map, ref_points_map, ref_points_vec);
  get_ref_line->getAppendixAttributeFromFile2Map(appendix_attribute_name, appendix_attribute_map);
  get_ref_line->getBoundrayCircleFromFile2Map(pysical_circle_name, boundray_circle_map);
  get_ref_line->getBoundrayLineFromFile2Map(pysical_line_name, boundray_line_map);
  get_ref_line->getBoundrayPointsFromFile2Ver(pysical_points_name, boundray_points_vec);
  get_ref_line->showALLRefPointsFromMap(ref_points_map);

  // get_ref_line->getRefLineFromFile2Map(bj_line_file_name, bj_line_map);
  // get_ref_line->getRefPointFromFile2Map(bj_point_file_name, bj_line_map, bj_points_map, bj_points_vec);

  rvizAllPointPubNewMap();
  rvizAllPointPubBoundrayPoints();
  rvizAllMapAppendixAttributePub();
  // rvizBJPointPubNewMap();
}

RefSender::~RefSender()
{
  ROS_INFO("RefSender release!");
}

void RefSender::refSenderInit()
{
  get_csv_tip = 0;
  route_recv_tip = 0;
  vcu_location_recv_tip = 0;
  task_click_recv_tip = 0;
  find_vcu_laneID_tip = 0;
  find_laneID_ref_tip = 0;
  send_planing_ = 0;
  send_planing_first_ = 0;
  last_id_ = 0;
  next_id_ = 0;
  now_id_ = 0;
  route_last_ret_ = 0;
  hmi_cmd_type_ = 0;
  hmi_cmd_type_recv_tip = 0;
  hmi_cmd_num_ = 0;
  ultra_status_ = 0;
  ultra_recv_tip = 0;
  running_status_ = 0;
  ad_status_recv_tip = 0;
}

void RefSender::recvFusionLocationCallback(const location_msgs::FusionDataInfoConstPtr &msg)
{
  vcu_location_temp_.heading = msg->yaw;
}
void RefSender::recvADStatusCallback(const hmi_msgs::ADStatusConstPtr &msg)
{
  running_status_ = msg->running_status;  // 1静止
  ad_status_recv_tip = 1;
  ROS_INFO("rcev ad status is %d", running_status_);
}

void RefSender::recvUltraInfoCallback(const perception_sensor_msgs::UltrasonicInfoConstPtr &msg)
{
  //收到超声波传感器状态信息为0时可以启动
  if (msg->obstacle_num == 0)
  {
    ultra_status_ = 1;
  }
  else
  {
    //当超声波有状态时，运动状态下可启动
    switch (running_status_)
    {
      case 0:
        ultra_status_ = 0;
        ROS_INFO("rcev ultra status is %d  sensor num %u during AD Stop", ultra_status_, msg->obstacle_num);
        break;
      case 1:
        ultra_status_ = 1;
        ROS_INFO("rcev ultra status is %d  during Car Running", ultra_status_);
        break;
      case 2:
        ultra_status_ = 1;
        ROS_INFO("rcev ultra status is %d  during Car Running", ultra_status_);
        break;
      case 3:
        ultra_status_ = 1;
        ROS_INFO("rcev ultra status is %d  during Car Running", ultra_status_);
        break;
      default:
        ultra_status_ = 0;
    }

    //发布超声波传感器值
    ultra_point_vec.clear();
    for (size_t i = 0; i < msg->obstacle_num; i++)
    {
      UltrasonicStatus ultra_point_temp;
      ultra_point_temp.id = msg->ult_obstacle[i].id;
      ultra_point_temp.distance = msg->ult_obstacle[i].distance * 0.05;
      ultra_point_temp.status = msg->ult_obstacle[i].status;
      ultra_point_vec.push_back(ultra_point_temp);
    }
    if (ultra_point_vec.empty())
    {
    }
    else
    {
    }
  }
  ultra_recv_tip = 1;
}

// void RefSender::ultraPointPub()
// {
//   visualization_msgs::Marker ultra_points_normal;
//   visualization_msgs::Marker ultra_points_error;
//   visualization_msgs::Marker ultra_points_sensor;

//   ultra_points_normal.header.frame_id    = "/odom";
//   ultra_points_normal.header.stamp       = ros::Time::now();
//   ultra_points_normal.ns                 = "/map/all_ultra_normal_point";
//   ultra_points_normal.action             = visualization_msgs::Marker::ADD;
//   ultra_points_normal.pose.orientation.w = 1.0;
//   ultra_points_normal.id                 = 20;
//   ultra_points_normal.type               = visualization_msgs::Marker::POINTS;
//   // POINTS markers use x and y scale for width/height respectively
//   ultra_points_normal.scale.x = 0.2;
//   ultra_points_normal.scale.y = 0.2;
//   // Points are green
//   ultra_points_normal.color.r = 0;
//   ultra_points_normal.color.g = 0;
//   ultra_points_normal.color.b = 1;
//   ultra_points_normal.color.a = 1.0;

//   ultra_points_error.header.frame_id    = "/odom";
//   ultra_points_error.header.stamp       = ros::Time::now();
//   ultra_points_error.ns                 = "/map/all_ultra_error_point";
//   ultra_points_error.action             = visualization_msgs::Marker::ADD;
//   ultra_points_error.pose.orientation.w = 1.0;
//   ultra_points_error.id                 = 21;
//   ultra_points_error.type               = visualization_msgs::Marker::POINTS;
//   // POINTS markers use x and y scale for width/height respectively
//   ultra_points_error.scale.x = 0.3;
//   ultra_points_error.scale.y = 0.3;
//   // Points are green
//   ultra_points_error.color.r = 1;
//   ultra_points_error.color.g = 0;
//   ultra_points_error.color.b = 0;
//   ultra_points_error.color.a = 1.0;

//   ultra_points_sensor.header.frame_id    = "/odom";
//   ultra_points_sensor.header.stamp       = ros::Time::now();
//   ultra_points_sensor.ns                 = "/map/all_ultra_error_point";
//   ultra_points_sensor.action             = visualization_msgs::Marker::ADD;
//   ultra_points_sensor.pose.orientation.w = 1.0;
//   ultra_points_sensor.id                 = 22;
//   ultra_points_sensor.type               = visualization_msgs::Marker::POINTS;
//   // POINTS markers use x and y scale for width/height respectively
//   ultra_points_sensor.scale.x = 0.15;
//   ultra_points_sensor.scale.y = 0.15;
//   // Points are green
//   ultra_points_sensor.color.r = 0;
//   ultra_points_sensor.color.g = 1;
//   ultra_points_sensor.color.b = 0;
//   ultra_points_sensor.color.a = 1.0;

//   ROS_INFO("Map size %d", ultra_point_vec.size());
//   if (std::vector< UltrasonicStatus >::iterator it_loop = ultra_point_vec.begin(); it_loop != ultra_point_vec.end();
//       it_loop++)
//   {
//     //根据当前位置计算当前超声波传感器基础坐标

//     if (it_loop->status == 1)
//     {
//       geometry_msgs::Point p_error;
//       p_error.x = 1;
//       p_error.y = 1;
//       p_error.z = 0;
//       ultra_points_error.points.push_back(p_error);
//     }
//     else
//     {
//       geometry_msgs::Point p_normal;
//       p_normal.x = 1;
//       p_normal.y = 1;
//       p_normal.z = 0;
//       ultra_points_normal.push_back(p_normal);
//       if (it_loop->distance < 63)
//       {
//         for (size_t i = 0; i < 13; i++)
//         {
//           geometry_msgs::Point p_point;
//           float s_distanc = it_loop->distance * 0.05;
//           double s_theta  = (-60 + i * 10) * M_PI / 180;
//           p_point.x       = p_normal.x + s_distanc * cos(s_theta);
//           p_point.y       = p_normal.y + s_distanc * sin(s_theta);
//           p_point.y       = 0;
//           ultra_points_sensor.points.push_back(p_point);
//         }
//       }
//     }
//   }

//   for (std::map< int, std::vector< RefPoints > >::iterator it_loop_first = ref_points_map.begin();
//        it_loop_first != ref_points_map.end(); it_loop_first++)
//   {
//     ROS_INFO("vector %d size %d", it_loop_first->first, it_loop_first->second.size());
//     for (std::vector< RefPoints >::iterator it_loop_second = it_loop_first->second.begin();
//          it_loop_second != it_loop_first->second.end(); it_loop_second++)
//     {
//       geometry_msgs::Point p;
//       p.x = it_loop_second->point.x - ZJ_NEW_MAP_X;
//       p.y = it_loop_second->point.y - ZJ_NEW_MAP_Y;
//       p.z = it_loop_second->point.z;
//       ref_points.points.push_back(p);
//       // cout << "The " << loop_index << " at line " << it_loop_first->first << " point id："
//       //      << it_loop_second->ref_point_id << " (" << it_loop_second->point.x << ", " << it_loop_second->point.y
//       <<
//       //      ", "
//       //      << it_loop_second->point.z << " )" << endl;
//       ++loop_index;
//     }
//   }
//   rviz_all_point_pub_.publish(ref_points);
//   ROS_INFO("rviz pub");
// }

void RefSender::recvHMIControlADCallback(const hmi_msgs::HMIControlADConstPtr &msg)
{
  hmi_cmd_type_ = msg->CMD_Type;
  hmi_cmd_type_recv_tip = 1;
  hmi_cmd_num_++;
  ROS_INFO("rcev HMI msg %u", msg->CMD_Type);
  switch (hmi_cmd_type_)
  {
    case 0:
      ROS_INFO("rcev task num:%d type:%u NULL", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 1:
      ROS_INFO("rcev task num:%d type:%u Start AD", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 2:
      ROS_INFO("rcev task num:%d type:%u Lift Up", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 3:
      ROS_INFO("rcev task num:%d type:%u Lift Down", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 4:
      ROS_INFO("rcev task num:%d type:%u Charging", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 5:
      ROS_INFO("rcev task num:%d type:%u Stop Charge", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 6:
      ROS_INFO("rcev task num:%d type:%u Stop", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 7:
      ROS_INFO("rcev task num:%d type:%u Reset", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 8:
      ROS_INFO("rcev task num:%d type:%u Task Pause", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 9:
      ROS_INFO("rcev task num:%d type:%u Task continue", hmi_cmd_num_, hmi_cmd_type_);
      break;
    case 10:
      ROS_INFO("rcev task num:%d type:%u Task Cancel", hmi_cmd_num_, hmi_cmd_type_);
      break;
    default:
      ROS_INFO("rcev task num:%d type:%u ERROR ORDER", hmi_cmd_num_, hmi_cmd_type_);
      /* Default Code */
  }
}

void RefSender::recvClickCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  task_click_.x = msg->point.x;
  task_click_.y = msg->point.y;
  task_click_.laneID = (int)msg->point.z;
  task_click_recv_tip = 1;
  ROS_INFO("rcev rviz click: (%lf,%lf) in lane %d", task_click_.x, task_click_.y, task_click_.laneID);
}

void RefSender::recvVCUCallback(const location_msgs::FusionDataInfoConstPtr &msg)
{
  if (find_vcu_laneID_tip == 0)
  {
    vcu_location_.x = msg->pose.x;
    vcu_location_.y = msg->pose.y;
    vcu_location_.heading = msg->yaw;
    vcu_location_.laneID = 0;
    vcu_location_recv_tip = 1;

    // vcu_location_vec.push_back(vcu_location_);
    //    ROS_INFO("rcev vcu: (%lf,%lf) in lane %d", vcu_location_.x, vcu_location_.y, vcu_location_.laneID);
  }
}

void RefSender::recvRouteLaneIDArrayCallback(const std_msgs::Int64MultiArrayConstPtr &msg)
{
  if (find_laneID_ref_tip == 0)
  {
    route_data_.vcu_ID = 40;
    route_data_.data_status = 1;
    route_data_.data_length = msg->layout.dim[0].size;

    route_data_.data.clear();
    printf("recv: %d lane ", route_data_.data_length);
    for (size_t loop_i = 0; loop_i < route_data_.data_length; loop_i++)
    {
      int data_temp_ = msg->data[loop_i];
      route_data_.data.push_back(data_temp_);
      printf("%d ", data_temp_);
    }
    printf("\n");
    if (route_data_.vcu_ID == 40)
    {
      if (route_data_.data_status == 1)
      {
        route_recv_tip = 1;
      }
    }
    else
    {
      ROS_INFO("Recv wrong vcu route task!!!");
    }
  }
  else
  {
    ROS_INFO("Finding LaneID...");
  }
}

// -PI ~ PI --->>>  0 ~ 2PI
double RefSender::angleChange(const double theta_in_)
{
  double theta_out_ = 90 - (theta_in_ * 180 / M_PI);  //弧度 -> 角度
  if (theta_out_ < 0)
  {
    theta_out_ += 360;
  }
  return theta_out_;
}

void RefSender::taskPointPub(const TaskPoint &t_point_)
{
  hmi_msgs::VMSControlAD cmd_msg_;

  cmd_msg_.header.stamp = ros::Time::now();

  if (hmi_cmd_type_recv_tip == 1)
  {
    if (ultra_status_ == 0 && hmi_cmd_type_ == 1)
    {
      cmd_msg_.CMD_Type = 0;
    }
    else
    {
      //超声无障碍物或者运动状态或者其他命令
      cmd_msg_.CMD_Type = hmi_cmd_type_;
    }
  }
  else
  {
    cmd_msg_.CMD_Type = 0;
  }
  cmd_msg_.task_ID = hmi_cmd_num_;
  cmd_msg_.CMD_Type = hmi_cmd_type_;

  cmd_msg_.target_position.x = t_point_.x;
  cmd_msg_.target_position.y = t_point_.y;

#if ENABLE_ZJ_NEW_MAP
  cmd_msg_.target_position.heading = t_point_.heading;
#else
  cmd_msg_.target_position.heading = angleChange(t_point_.heading);
#endif

  vms_info_pub_.publish(cmd_msg_);

  ROS_INFO("Task cmd pub %u num %u ok,heading %lf", cmd_msg_.CMD_Type, cmd_msg_.task_ID, t_point_.heading);
}

void RefSender::routeDecisionInfoPub(const vector<RefLaneData> &rrps)
{
  plan_msgs::DecisionInfo decision_info;
  decision_info.header.stamp = ros::Time::now();
  decision_info.header.frame_id = "route_decision_info";

  int max_num_ = static_cast<int>(rrps.size());

  // double max_lane_speed  = 3.0;
  double max_lane_speed = 1.5;
  double min_lane_speed = 1.5;
  double max_curve_speed = 1.0;
  double min_curve_speed = 1.0;
  double begin_speed = 0.2;
  double end_speed = 0.1;
  double end_begin_speed = 0.1;
  double negative_acc = -3.0;  //负加速度
  double positive_acc = 1.0;   //正加速度
  double begin_acc = 0.2;

  int begin_tip = 1;
  int cur_tip = 0;
  int forward_tip = 0;
  int end_tip = 0;
  int forward_judge_num = 6;
  int end_point_judge_num = 9;
  int find_vcu_location = 0;

  int lane_keep_tip = 0;
  int lane_keep_begin_num = 0;
  int lane_keep_end_num = 0;

  common_msgs::PathPoint cur_path_point;
  common_msgs::PathPoint last_path_point;
  common_msgs::PathPoint end_path_point;
  common_msgs::PathPoint lane_keep_point_begint;
  common_msgs::PathPoint lane_keep_point_end;

  lane_keep_point_begint.x = -32.9989;
  lane_keep_point_begint.y = 63.0728;
  lane_keep_point_begint.v = 0;
  lane_keep_point_end.x = -8.62514;
  lane_keep_point_end.y = 93.0093;
  lane_keep_point_end.v = 0;

  end_path_point.x = task_click_.x;
  end_path_point.y = task_click_.y;
  end_path_point.v = 0;

  int init_first = 0;
  double distance_ref = 0.0;
  int min_index = 0;
  double min_d = 100;
  int min_end_index = 0;
  double min_end_d = 100;
  int use_lane_keep = 0;

  for (int i = 0; i < max_num_; i++)
  {
    RefLaneData ref_lane_data_temp = rrps.at(i);
    distance_ref =
        pointDistanceSquare(ref_lane_data_temp.point_x, ref_lane_data_temp.point_y, vcu_location_.x, vcu_location_.y);
    if (min_d > distance_ref)
    {
      min_index = i;
      min_d = distance_ref;
    }
    distance_ref =
        pointDistanceSquare(ref_lane_data_temp.point_x, ref_lane_data_temp.point_y, end_path_point.x, end_path_point.y);
    if (min_end_d > distance_ref)
    {
      min_end_index = i;
      min_end_d = distance_ref;
    }

    if (use_lane_keep == 1)
    {
      distance_ref = pointDistanceSquare(ref_lane_data_temp.point_x, ref_lane_data_temp.point_y,
                                         lane_keep_point_begint.x, lane_keep_point_begint.y);
      if (distance_ref < (0.5 * 0.5))
      {
        lane_keep_begin_num = i;
      }
      distance_ref = pointDistanceSquare(ref_lane_data_temp.point_x, ref_lane_data_temp.point_y, lane_keep_point_end.x,
                                         lane_keep_point_end.y);
      if (distance_ref < (0.5 * 0.5))
      {
        lane_keep_end_num = i;
      }
    }
  }

  if (use_lane_keep == 1 && lane_keep_end_num > lane_keep_begin_num)
  {
    lane_keep_tip = 1;
  }

  if (min_index != 0)
  {
    ROS_ERROR("AGV    point is (%lf, %lf) at %d min distance %lf", vcu_location_.x, vcu_location_.y, min_index, min_d);
  }
  else
  {
    ROS_ERROR("AGV    point is (%lf, %lf) not find", vcu_location_.x, vcu_location_.y);
  }
  if (min_end_index != 0)
  {
    ROS_ERROR("Target point is (%lf, %lf) at %d min distance %lf", end_path_point.x, end_path_point.y, min_end_index,
              min_end_d);
  }
  else
  {
    ROS_ERROR("Target point is (%lf, %lf) not find", end_path_point.x, end_path_point.y);
  }

  for (int i = 0; i < max_num_; i++)
  {
    RefLaneData ref_lane_data_temp = rrps.at(i);
    RefLaneData forward_ref_lane_data_temp;
    RefLaneData end_point_data_temp;

    if (init_first == 0)
    {
      init_first = 1;
      last_path_point.x = ref_lane_data_temp.point_x;
      last_path_point.y = ref_lane_data_temp.point_y;
      last_path_point.v = begin_speed;
    }

    double angle_temp = ref_lane_data_temp.theta;
    cur_path_point.x = ref_lane_data_temp.point_x;
    cur_path_point.y = ref_lane_data_temp.point_y;

    //转弯角度判断 针对0821s镇江港地图修订
    // if (ref_lane_data_temp.kappa == 0 && ref_lane_data_temp.dappa == 0)
    // {
    //   angle_temp = angle_temp;
    // }
    // else
    // {
    //   angle_temp = 2 * M_PI - angle_temp;
    //   if (angle_temp < 0)
    //   {
    //     angle_temp = angle_temp + 2 * M_PI;
    //   }
    // }

    cur_path_point.theta = angle_temp * 180 / M_PI;  // 2PI -> 360;

    //斜行判断
    // std::map< int, RefLine > ref_line_map;
    std::map<int, RefLine>::iterator ref_line_map_it;
    ref_line_map_it = ref_line_map.find(ref_lane_data_temp.laneID);
    if (ref_line_map_it == ref_line_map.end())
    {
      ROS_ERROR("can not find this lane id %d from map", ref_lane_data_temp.laneID);
    }
    else
    {
      if (ref_line_map_it->second.line_direction == 3)
      {
        cur_path_point.theta = last_path_point.theta;
      }
      if (ref_line_map_it->second.line_direction == 5)
      {
        cur_path_point.theta = last_path_point.theta;
      }
    }

    // if (ref_lane_data_temp.laneID == 36)
    // {
    //   cur_path_point.theta = last_path_point.theta;
    // }

    // if (ref_lane_data_temp.laneID == 61)
    // {
    //   cur_path_point.theta = last_path_point.theta;
    // }

    // if (ref_lane_data_temp.laneID == 35)
    // {
    //   cur_path_point.theta = last_path_point.theta;
    // }

    cur_path_point.v = last_path_point.v;

    double distance_point =
        pointDistanceSquare(cur_path_point.x, cur_path_point.y, last_path_point.x, last_path_point.y);
    double end_point_distance = 0.0;
    //当前车道判断 前方车道 终点判断
    int k_forward_ = i;
    int k_end_ = i;

    if ((k_forward_ + end_point_judge_num) < max_num_)
    {
      k_forward_ = k_forward_ + forward_judge_num;
      k_end_ = k_end_ + end_point_judge_num;

      forward_ref_lane_data_temp = rrps.at(k_forward_);
      end_point_data_temp = rrps.at(k_end_);

      if (ref_lane_data_temp.kappa == 0 && ref_lane_data_temp.dappa == 0)
      {
        cur_tip = 0;  //直道
      }
      else
      {
        cur_tip = 1;  //弯道
      }

      if (forward_ref_lane_data_temp.kappa == 0 && forward_ref_lane_data_temp.dappa == 0)
      {
        forward_tip = 0;
      }
      else
      {
        forward_tip = 1;
      }

      end_point_distance = pointDistanceSquare(end_point_data_temp.point_x, end_point_data_temp.point_y,
                                               end_path_point.x, end_path_point.y);
      if (end_point_distance < (0.5 * 0.5))
      {
        end_tip = 1;
        end_begin_speed = cur_path_point.v;
      }
    }
    // ROS_INFO("Now ref status is cur_tip:%d forward_tip:%d begin_tip:%d end_tip:%d", cur_tip, forward_tip, begin_tip,
    //          end_tip);
    //启动
    if (begin_tip == 1)
    {
      // ROS_INFO("find_vcu_location %d i:%d min_index:%d", find_vcu_location, i, min_index);
      if (find_vcu_location == 0)
      {
        if (i < min_index)
        {
          cur_path_point.v = begin_speed;
        }
        else
        {
          find_vcu_location = 1;
          cur_path_point.v = begin_speed;
        }
      }
      else
      {
        if (last_path_point.v == 0)
        {
          cur_path_point.v = last_path_point.v + begin_acc * (sqrt(distance_point) / begin_speed);
        }
        else
        {
          cur_path_point.v = last_path_point.v + begin_acc * (sqrt(distance_point) / last_path_point.v);
        }

        if (cur_tip == 0)
        {
          if (cur_path_point.v > max_lane_speed)
          {
            cur_path_point.v = max_lane_speed;
            begin_tip = 0;
          }
        }
        else
        {
          if (cur_path_point.v > max_curve_speed)
          {
            cur_path_point.v = max_curve_speed;
            begin_tip = 0;
          }
        }
      }
    }
    else
    {
      //弯道保持
      if (cur_tip == 1)
      {
        if (cur_path_point.v > max_curve_speed)
        {
          cur_path_point.v = max_curve_speed;
        }
        if (cur_path_point.v < min_curve_speed)
        {
          cur_path_point.v = min_curve_speed;
        }
      }
      else
      {
        //直道加速减速
        if (forward_tip == 0)
        {
          //加速
          if (positive_acc == 1)
          {
            if (cur_path_point.v < max_lane_speed)
            {
              cur_path_point.v = last_path_point.v + positive_acc * (sqrt(distance_point) / last_path_point.v);
              if (cur_path_point.v > max_lane_speed)
              {
                cur_path_point.v = max_lane_speed;
                positive_acc = 0;
              }
            }
            else
            {
              cur_path_point.v = last_path_point.v;
              positive_acc = 0;
            }
          }
          else  //减速
          {
            if (cur_path_point.v > min_lane_speed)
            {
              cur_path_point.v = last_path_point.v + negative_acc * (sqrt(distance_point) / last_path_point.v);
              if (cur_path_point.v < min_lane_speed)
              {
                cur_path_point.v = min_lane_speed;
                positive_acc = 1;
              }
            }
            else
            {
              cur_path_point.v = last_path_point.v;
              positive_acc = 1;
            }
          }
        }
        //直道进弯道 //入弯减速
        else
        {
          cur_path_point.v = last_path_point.v + negative_acc * (sqrt(distance_point) / last_path_point.v);
          if (cur_path_point.v < max_curve_speed)
          {
            cur_path_point.v = max_curve_speed;
          }
        }
      }
    }

    //速度保持
    if (lane_keep_tip == 1)
    {
      if (i > lane_keep_begin_num)
      {
        double last_cur_speed = cur_path_point.v;
        cur_path_point.v = 3.0;
        if (i > lane_keep_end_num)
        {
          cur_path_point.v = last_cur_speed;
        }
      }
    }

    //到达终点减速
    if (end_tip == 1)
    {
      cur_path_point.v = last_path_point.v - end_begin_speed / end_point_judge_num;
      if (cur_path_point.v < 0)
      {
        cur_path_point.v = 0;
      }
      //终点后
      if (i > min_end_index)
      {
        cur_path_point.v = 0;
      }
    }
    // ROS_WARN("ref ID %d Point %d %lf (%lf,%lf) ref_heading:%lf -ref:%lf  taget_Speed:%lf last_heading %lf lane_dir
    // %d",
    //          ref_lane_data_temp.laneID, i, ref_lane_data_temp.d_from_begin, cur_path_point.x, cur_path_point.y,
    //          cur_path_point.theta, ref_lane_data_temp.theta, cur_path_point.v, last_path_point.theta,
    //          ref_line_map_it->second.line_direction);
    // ROS_WARN("Point %d %lf (%lf,%lf) taget_Speed:%lf end_d:%lf cur-last_distance:%lf speed+:%lf", i,
    //          ref_lane_data_temp.d_from_begin, cur_path_point.x, cur_path_point.y, cur_path_point.v,
    //          end_point_distance,
    //          distance_point, begin_acc * (sqrt(distance_point) / last_path_point.v));
    decision_info.path_data_REF.push_back(cur_path_point);

    last_path_point.x = cur_path_point.x;
    last_path_point.y = cur_path_point.y;
    last_path_point.v = cur_path_point.v;
    last_path_point.theta = cur_path_point.theta;
  }
  decision_info.path_plan_valid = 1;
  decision_info.path_mode = 1;

  // route_decision_info_pub_.publish(decision_info);

  //起步10m 0.5起步
  //结束10m 2m ->> 0m
}

void RefSender::routeRefPlanningPub(const vector<RefLaneData> &rrps)
{
  map_msgs::REFPointArray ref_pinfo;
  ref_pinfo.header.stamp = ros::Time::now();
  ref_pinfo.header.frame_id = "route_ref_infor";
  //  ref_pinfo.target_lane_ID.data = (uint16_t)task_click_.laneID;
  //  ref_pinfo.target_lane_ID = ( uint32_t )rrps[rrps.size() - 1].laneID;
  ref_pinfo.target_lane_ID = (uint32_t)rrps.rbegin()->laneID;

  //  ref_pinfo.agv_lane_ID = ( uint32_t )vcu_location_.laneID;
  //  ref_pinfo.agv_lane_ID = ( uint32_t )rrps[0].laneID;
  ref_pinfo.agv_lane_ID = (uint32_t)rrps.begin()->laneID;

  double last_rkappa = 0;
  double last_ds = 0;
  //  for (vector< RefLaneData >::iterator loop_index = rrps.begin(); loop_index != rrps.end(); loop_index++)
  for (size_t i = 0; i < rrps.size(); i++)
  {
    common_msgs::REFPoint ref_point_;
    RefLaneData ref_lane_data_temp = rrps.at(i);

    int go_next_tip = 0;

    ref_point_.rs = ref_lane_data_temp.d_from_begin;  // m
    ref_point_.rx = ref_lane_data_temp.point_x;       // m
    ref_point_.ry = ref_lane_data_temp.point_y;       // m

    double angle_temp_ = ref_lane_data_temp.theta;
    ref_point_.rtheta = angle_temp_ * 180 / M_PI;  // 2PI -> 360;
    ref_point_.rkappa = ref_lane_data_temp.kappa;  // 1/m

    //计算曲率变化率
    if (i > 0)
    {
      if (ref_point_.rs != last_ds)
      {
        ref_point_.rdkappa = (ref_point_.rkappa - last_rkappa) / (ref_point_.rs - last_ds);  // 1/m2
      }
      else
      {
        if (ref_point_.rkappa != last_rkappa)
        {
          // go_next_tip = 1;
          ref_point_.rdkappa = 0;
        }
        else
        {
          ref_point_.rdkappa = 0;
        }
      }
    }
    else
    {
      ref_point_.rdkappa = 0;  // 1/m2
    }
    //计算曲率变化率

    if (go_next_tip == 0)
    {
      last_ds = ref_point_.rs;
      last_rkappa = ref_point_.rkappa;

      ref_point_.max_speed = ref_lane_data_temp.max_speed;
      ref_point_.lane_id = ref_lane_data_temp.laneID;

      //斜行判断
      // std::map< int, RefLine > ref_line_map;
      std::map<int, RefLine>::iterator ref_line_map_it;
      ref_line_map_it = ref_line_map.find(ref_lane_data_temp.laneID);
      if (ref_line_map_it == ref_line_map.end())
      {
        ROS_ERROR("can not find this lane id %d from map", ref_lane_data_temp.laneID);
      }
      else
      {
        ref_point_.line_direction = ref_line_map_it->second.line_direction;
      }
      // ROS_INFO("REF ID %d direction %d (%lf,%lf)", ref_point_.lane_id, ref_point_.line_direction, ref_point_.rx,
      //          ref_point_.ry);
      //地图数据错误，先写死此值
      if (ref_point_.rkappa == 0 && ref_point_.rdkappa == 0)
      {
        ref_point_.max_speed = 8;
      }
      else
      {
        ref_point_.max_speed = 2;
      }

      for (std::vector<LineWidth>::iterator loop_i = ref_lane_data_temp.line_width.begin();
           loop_i != ref_lane_data_temp.line_width.end(); loop_i++)
      {
        common_msgs::LaneRange lane_range_;
        lane_range_.left_boundary = loop_i->left;
        lane_range_.right_boundary = loop_i->right;
        ref_point_.lane_ranges.push_back(lane_range_);
      }

      ROS_INFO("Pub Ref:%d direction %d ds:%lf (%lf,%lf) theta:%lf kappa:%lf dkappa:%lf", ref_point_.lane_id,
               ref_point_.line_direction, ref_point_.rs, ref_point_.rx, ref_point_.ry, ref_point_.rtheta,
               ref_point_.rkappa, ref_point_.rdkappa);

      ref_pinfo.REF_line_INFO.push_back(ref_point_);
    }
  }
  route_ref_planning_pub_.publish(ref_pinfo);
  // ROS_INFO("Pub %u -> %u total %d ref point", ref_pinfo.agv_lane_ID,
  //          ref_pinfo.target_lane_ID, rrps.size());
}

void RefSender::setRefLineInit()
{
  if (!ref_line_map.empty())
  {
    ref_line_map.clear();
  }
  if (!ref_points_map.empty())
  {
    ref_points_map.clear();
  }
  if (!appendix_attribute_map.empty())
  {
    appendix_attribute_map.clear();
  }
  if (!ref_points_vec.empty())
  {
    ref_points_vec.clear();
  }
}

void RefSender::rvizVcuPointPubTest()
{
  geometry_msgs::PointStamped psmsg_gps;
  psmsg_gps.header.stamp = ros::Time::now();
  psmsg_gps.header.frame_id = "odom";
  psmsg_gps.point.x = vcu_location_.x;
  psmsg_gps.point.y = vcu_location_.y;
  psmsg_gps.point.z = 0;
  rviz_point_pub_.publish(psmsg_gps);
}

void RefSender::rvizVCUPointPubNewMap()
{
  visualization_msgs::Marker vcu_points;
  int loop_index = 0;

  vcu_points.header.frame_id = "/odom";
  vcu_points.header.stamp = ros::Time::now();
  vcu_points.ns = "/map/all_vcu_point";
  vcu_points.action = visualization_msgs::Marker::ADD;
  vcu_points.pose.orientation.w = 1.0;
  vcu_points.id = 3;
  vcu_points.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  vcu_points.scale.x = 0.2;
  vcu_points.scale.y = 0.2;
  // Points are green
  vcu_points.color.r = 0;
  vcu_points.color.g = 1;
  vcu_points.color.b = 0;
  vcu_points.color.a = 1.0;

  ROS_INFO("Map size %d", vcu_location_vec.size());

  for (std::vector<TaskPoint>::iterator loop_index = vcu_location_vec.begin(); loop_index != vcu_location_vec.end();
       loop_index++)
  {
    geometry_msgs::Point p;
    p.x = loop_index->x - ZJ_NEW_MAP_X;
    p.y = loop_index->y - ZJ_NEW_MAP_Y;
    p.z = 0;
    vcu_points.points.push_back(p);
    // cout << "The " << loop_index << " at line " << it_loop_first->first << " point id："
    //      << it_loop_second->ref_point_id << " (" << it_loop_second->point.x << ", " << it_loop_second->point.y <<
    //      ", "
    //      << it_loop_second->point.z << " )" << endl;
  }
  rviz_vcu_point_pub_.publish(vcu_points);
  ROS_INFO("rviz pub");
}

void RefSender::rvizBJPointPubNewMap()
{
  visualization_msgs::Marker bj_points;
  int loop_index = 0;

  bj_points.header.frame_id = "/odom";
  bj_points.header.stamp = ros::Time::now();
  bj_points.ns = "/map/all_bj_point";
  bj_points.action = visualization_msgs::Marker::ADD;
  bj_points.pose.orientation.w = 1.0;
  bj_points.id = 2;
  bj_points.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  bj_points.scale.x = 0.2;
  bj_points.scale.y = 0.2;
  // Points are green
  bj_points.color.r = 0;
  bj_points.color.g = 0;
  bj_points.color.b = 1;
  bj_points.color.a = 1.0;

  ROS_INFO("Map size %d", bj_points_map.size());

  for (std::map<int, std::vector<RefPoints> >::iterator it_loop_first = bj_points_map.begin();
       it_loop_first != bj_points_map.end(); it_loop_first++)
  {
    ROS_INFO("vector %d size %d", it_loop_first->first, it_loop_first->second.size());
    for (std::vector<RefPoints>::iterator it_loop_second = it_loop_first->second.begin();
         it_loop_second != it_loop_first->second.end(); it_loop_second++)
    {
      geometry_msgs::Point p;
      p.x = it_loop_second->point.x - ZJ_NEW_MAP_X;
      p.y = it_loop_second->point.y - ZJ_NEW_MAP_Y;
      p.z = it_loop_second->point.z;
      bj_points.points.push_back(p);
      // cout << "The " << loop_index << " at line " << it_loop_first->first << " point id："
      //      << it_loop_second->ref_point_id << " (" << it_loop_second->point.x << ", " << it_loop_second->point.y <<
      //      ", "
      //      << it_loop_second->point.z << " )" << endl;
      ++loop_index;
    }
  }
  rviz_bj_point_pub_.publish(bj_points);
  ROS_INFO("rviz pub");
}

void RefSender::rvizColorTest()
{
  visualization_msgs::MarkerArray color_array;
  int loop_index = 0;
  for (size_t r_index = 0; r_index <= 10; r_index++)
  {
    for (size_t g_index = 0; g_index <= 10; g_index++)
    {
      for (size_t b_index = 0; b_index <= 10; b_index++)
      {
        ++loop_index;
        visualization_msgs::Marker color_point;
        color_point.header.frame_id = "/odom";
        color_point.header.stamp = ros::Time::now();
        color_point.ns = "/map/all_ref_point";
        color_point.action = visualization_msgs::Marker::ADD;
        color_point.pose.orientation.w = 1.0;
        color_point.id = loop_index;
        color_point.type = visualization_msgs::Marker::POINTS;
        // POINTS markers use x and y scale for width/height respectively
        color_point.scale.x = 0.3;
        color_point.scale.y = 0.3;
        // Points are green
        color_point.color.r = (float)r_index / 10;
        color_point.color.g = (float)g_index / 10;
        color_point.color.b = (float)b_index / 10;
        color_point.color.a = 1.0;
        geometry_msgs::Point p;
        p.x = g_index + 11 * r_index;
        p.y = b_index;
        p.z = 0;
        color_point.points.push_back(p);
        color_array.markers.push_back(color_point);
      }
    }
  }

  rviz_color_test_pub_.publish(color_array);
  ROS_INFO("pub color test");
}

void RefSender::rvizAllMapAppendixAttributePub()
{
  visualization_msgs::MarkerArray aa_array;
  std::map<int, AppendixAttribute>::iterator itor_aa;
  int loop_ = 0;
  for (itor_aa = appendix_attribute_map.begin(); itor_aa != appendix_attribute_map.end(); ++itor_aa)
  {
    visualization_msgs::Marker aa_point;
    aa_point.header.frame_id = "/odom";
    aa_point.header.stamp = ros::Time::now();
    aa_point.ns = "/map/all_aa_point";
    aa_point.action = visualization_msgs::Marker::ADD;
    aa_point.pose.orientation.w = 1.0;
    aa_point.id = loop_;
    aa_point.type = visualization_msgs::Marker::LINE_STRIP;
    // POINTS markers use x and y scale for width/height respectively
    aa_point.scale.x = 0.2;
    aa_point.scale.y = 0.2;
    // Points are green
    aa_point.color.r = 0.8;
    aa_point.color.g = 0;
    aa_point.color.b = 0;
    aa_point.color.a = 1.0;

    geometry_msgs::Point p;
    for (size_t i = 0; i < itor_aa->second.corner_count; ++i)
    {
      p.x = itor_aa->second.corner_point.at(i).x;
      p.y = itor_aa->second.corner_point.at(i).y;
      p.z = itor_aa->second.corner_point.at(i).z;
      aa_point.points.push_back(p);
    }
    p.x = itor_aa->second.corner_point.at(0).x;
    p.y = itor_aa->second.corner_point.at(0).y;
    p.z = itor_aa->second.corner_point.at(0).z;
    aa_point.points.push_back(p);

    aa_array.markers.push_back(aa_point);
    ++loop_;
  }

  rviz_appendix_attributemap_pub_.publish(aa_array);
  ROS_INFO("pub %d appendix attribute", loop_);
}

void RefSender::rvizAllPointPubBoundrayPoints()
{
  visualization_msgs::Marker ref_points;
  int loop_index = 0;

  ref_points.header.frame_id = "/odom";
  ref_points.header.stamp = ros::Time::now();
  ref_points.ns = "/map/all_boundray_point";
  ref_points.action = visualization_msgs::Marker::ADD;
  ref_points.pose.orientation.w = 1.0;
  ref_points.id = 2;
  ref_points.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  ref_points.scale.x = 0.2;
  ref_points.scale.y = 0.2;
  // Points are green
  ref_points.color.r = 0;
  ref_points.color.g = 0;
  ref_points.color.b = 1;
  ref_points.color.a = 1.0;

  ROS_INFO("boundray_points_vec size %d", boundray_points_vec.size());
  for (size_t i = 0; i < boundray_points_vec.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = boundray_points_vec.at(i).point.x;
    p.y = boundray_points_vec.at(i).point.y;
    p.z = boundray_points_vec.at(i).point.z;
    ref_points.points.push_back(p);
  }
  rviz_all_point_pub_.publish(ref_points);
  sleep(1);
}

void RefSender::rvizAllPointPubNewMap()
{
  visualization_msgs::Marker ref_points;
  int loop_index = 0;

  ref_points.header.frame_id = "/odom";
  ref_points.header.stamp = ros::Time::now();
  ref_points.ns = "/map/all_ref_point";
  ref_points.action = visualization_msgs::Marker::ADD;
  ref_points.pose.orientation.w = 1.0;
  ref_points.id = 2;
  ref_points.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  ref_points.scale.x = 0.4;
  ref_points.scale.y = 0.4;
  // Points are green
  ref_points.color.r = 1;
  ref_points.color.g = 0;
  ref_points.color.b = 0;
  ref_points.color.a = 1.0;

  ROS_INFO("Map size %d", ref_points_map.size());

  double x_max = -100;
  double x_min = 100;

  double y_max = -100;
  double y_min = 100;

  for (std::map<int, std::vector<RefPoints> >::iterator it_loop_first = ref_points_map.begin();
       it_loop_first != ref_points_map.end(); it_loop_first++)
  {
    ROS_INFO("vector %d size %d", it_loop_first->first, it_loop_first->second.size());
    for (std::vector<RefPoints>::iterator it_loop_second = it_loop_first->second.begin();
         it_loop_second != it_loop_first->second.end(); it_loop_second++)
    {
      geometry_msgs::Point p;
      p.x = it_loop_second->point.x - ZJ_NEW_MAP_X;
      p.y = it_loop_second->point.y - ZJ_NEW_MAP_Y;
      p.z = it_loop_second->point.z;
      ref_points.points.push_back(p);

      x_max = max(x_max, p.x);
      x_min = min(x_min, p.x);
      y_max = max(y_max, p.y);
      y_min = min(y_min, p.y);
      // cout << "The " << loop_index << " at line " << it_loop_first->first << " point id："
      //      << it_loop_second->ref_point_id << " (" << it_loop_second->point.x << ", " << it_loop_second->point.y <<
      //      ", "
      //      << it_loop_second->point.z << " )" << endl;
      ++loop_index;
    }
  }
  rviz_all_point_pub_.publish(ref_points);
  ROS_INFO("rviz pub max(%lf,%lf) min(%lf,%lf)", x_max, y_max, x_min, y_min);
  sleep(1);
}

void RefSender::rvizAllPointPub()
{
  visualization_msgs::Marker ref_points;
  ref_points.header.frame_id = "/odom";
  ref_points.header.stamp = ros::Time::now();
  ref_points.ns = "/map/all_ref_point";
  ref_points.action = visualization_msgs::Marker::ADD;
  ref_points.pose.orientation.w = 1.0;
  ref_points.id = 1;
  ref_points.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  ref_points.scale.x = 0.2;
  ref_points.scale.y = 0.2;
  // Points are green
  ref_points.color.r = 1;
  ref_points.color.g = 0;
  ref_points.color.b = 1;
  ref_points.color.a = 1.0;

  std::map<int, std::vector<std::vector<double> > >::iterator ref_iter;
  for (ref_iter = ref_line_data.begin(); ref_iter != ref_line_data.end(); ref_iter++)
  {
    for (size_t i = 0; i < ref_iter->second.size(); i++)
    {
      geometry_msgs::Point p;
      p.x = ref_iter->second[i][1];
      p.y = ref_iter->second[i][2];
      p.z = 0;
      ref_points.points.push_back(p);
      i = i + 4;
    }
  }
  rviz_all_point_pub_.publish(ref_points);
  ROS_INFO("rviz pub");
}

void RefSender::rvizRouteRefPointPub(const vector<RefLaneData> &rrps)
{
  visualization_msgs::Marker ref_points;
  ref_points.header.frame_id = "/odom";
  ref_points.header.stamp = ros::Time::now();
  ref_points.ns = "/map/route_ref_point";
  ref_points.action = visualization_msgs::Marker::ADD;
  ref_points.pose.orientation.w = 1.0;
  ref_points.id = 2;
  ref_points.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  ref_points.scale.x = 0.5;
  ref_points.scale.y = 0.5;
  // Points are green
  ref_points.color.r = 1;
  ref_points.color.g = 0;
  ref_points.color.b = 0;
  ref_points.color.a = 1.0;
  for (size_t i = 0; i < rrps.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = rrps[i].point_x;
    p.y = rrps[i].point_y;
    p.z = 0;
    ref_points.points.push_back(p);
  }

  rviz_route_ref_pub_.publish(ref_points);
  ROS_INFO("route pub");
}

void RefSender::vcuLocationPub()
{
  geometry_msgs::PointStamped ps;
  ps.header.stamp = ros::Time::now();
  ps.header.frame_id = "vcu_Location_laneID";
  ps.point.x = vcu_location_.x;
  ps.point.y = vcu_location_.y;
  ps.point.z = vcu_location_.laneID;
  vcu_location_pub_.publish(ps);
  ROS_INFO("VCU laneID pub ok");
}

void RefSender::readRefFromCSV()
{
  char *home_path = getenv("HOME");
  char ref_line_name[1024] = { 0 };
  sprintf(ref_line_name, "%s" MAP_REF_POINT_CONNECT_PATH "" REF_LINE_NAME, home_path);
  ROS_INFO("ref_line name:%s", ref_line_name);

  int ref_line_data_key;
  vector<vector<double> > ref_line_data_value;

  ifstream infile;
  infile.open(ref_line_name);  //将文件流对象与文件连接起来
  assert(infile.is_open());    //若失败,则输出错误消息,并终止程序运行

  vector<double> line_read_double;
  int read_column = 0;  //列
  int read_row = 0;     //行
  char read_c = 0;      //字符
  char c;
  char read_buf_temp[128];
  int begin_tip = 0;
  while (!infile.eof())
  {
    infile.get(c);
    read_buf_temp[read_c] = c;
    read_c++;
    if (c == EOF)
    {
      break;
    }
    if (c == '\n')
    {
      read_c = 0;
      if (read_column > 0)
      {
        read_column = 0;
        int end_double_temp = atof(read_buf_temp);
        line_read_double.push_back(end_double_temp);
        ref_line_data_value.push_back(line_read_double);
        read_row++;
      }
      line_read_double.clear();
    }
    if (c == ',')
    {
      read_c = 0;

      if (read_column == 0)
      {
        int t_int_temp = atoi(read_buf_temp);
        if (begin_tip == 0)
        {
          begin_tip = 1;
        }
        else
        {
          if (ref_line_data_key != t_int_temp)
          {
            ref_line_data.insert(pair<int, vector<vector<double> > >(ref_line_data_key, ref_line_data_value));
            // ROS_INFO("map %d lane %d data %d", ref_line_data.size(),
            // ref_line_data_value.size(),
            //          ref_line_data_value[0].size());
            ref_line_data_value.clear();
          }
        }
        ref_line_data_key = t_int_temp;
        line_read_double.push_back(t_int_temp);
      }
      else
      {
        double t_double_temp = atof(read_buf_temp);
        line_read_double.push_back(t_double_temp);
      }
      read_column++;
    }
  }
  ref_line_data.insert(pair<int, vector<vector<double> > >(ref_line_data_key, ref_line_data_value));
  // ROS_INFO("map %d lane %d data %d", ref_line_data.size(),
  // ref_line_data_value.size(),
  // ref_line_data_value[0].size());
  ref_line_data_value.clear();

  infile.close();
  memset(read_buf_temp, 0, sizeof(read_buf_temp));
  line_read_double.clear();
  read_column = 0;  //列
  read_row = 0;     //行
  read_c = 0;       //字符

  std::map<int, std::vector<std::vector<double> > >::iterator ref_iter;
  // for (ref_iter = ref_line_data.begin(); ref_iter != ref_line_data.end();
  // ref_iter++)
  // {
  //   ROS_INFO("read map lane %d data %d ok!", ref_iter->first,
  //   ref_iter->second.size());
  // }
  ROS_INFO("read map ok!");
  get_csv_tip = 1;
}

//自定义排序函数
bool RefSender::sortFun(const MinRefPoint &d1, const MinRefPoint &d2)
{
  return d1.min_distance < d2.min_distance;  //升序排列
}

int RefSender::findLaneIDWithXY(TaskPoint &find_point_in_, int find_mode)
{
  int laneID_ = 0;

#if ENABLE_ZJ_NEW_MAP
  RefPoints ref_point_temp;
  ref_point_temp.point.x = find_point_in_.x + ZJ_NEW_MAP_X;
  ref_point_temp.point.y = find_point_in_.y + ZJ_NEW_MAP_Y;
  ref_point_temp.point.z = 0.0;

  if (find_vcu_laneID_tip == 1)
  {
    ref_point_temp.theta = find_point_in_.heading * M_PI / 180;
    laneID_ = get_ref_line->findLineFromRefPointVecWithXYZHeading(ref_point_temp, ref_points_vec);
    find_point_in_.d_from_line_begin = ref_point_temp.d_from_line_begin;
  }
  else
  {
    laneID_ = get_ref_line->findXYZFromRefPointVecTraversal(ref_point_temp, ref_points_vec);
    find_point_in_.heading = ref_point_temp.theta * 180 / M_PI;
    find_point_in_.d_from_line_begin = ref_point_temp.d_from_line_begin;
  }

  find_point_in_.laneID = laneID_;

  if (find_vcu_laneID_tip == 1)
  {
    if (find_point_in_.laneID == 0)
    {
      ROS_ERROR("Find (%lf,%lf) in line %d heading ref %lf  <-> vcu %lf", find_point_in_.x, find_point_in_.y,
                find_point_in_.laneID, ref_point_temp.theta * 180 / M_PI, find_point_in_.heading);
    }
    else
    {
      ROS_WARN("Find (%lf,%lf) in line %d heading ref %lf  <-> vcu %lf ds:%lf", find_point_in_.x, find_point_in_.y,
               find_point_in_.laneID, ref_point_temp.theta * 180 / M_PI, find_point_in_.heading,
               find_point_in_.d_from_line_begin);
    }
  }
  else
  {
    ROS_INFO("Find (%lf,%lf) in line %d heading %lf ds:%lf", find_point_in_.x, find_point_in_.y, find_point_in_.laneID,
             find_point_in_.heading, find_point_in_.d_from_line_begin);
  }

#else
  std::vector<MinRefPoint> min_distance_data_;
  std::map<int, std::vector<std::vector<double> > >::iterator iter;
  for (iter = ref_line_data.begin(); iter != ref_line_data.end(); iter++)
  {
    MinRefPoint mdd_temp_;
    int point_ser = mathDistance(find_point_in_.x, find_point_in_.y, iter->second, mdd_temp_);
    if (point_ser > -1)
    {
      min_distance_data_.push_back(mdd_temp_);
    }
  }
  if (min_distance_data_.size() > 0)
  {
    //按照数值从小到大排序,然后查找
    std::sort(min_distance_data_.begin(), min_distance_data_.end(), sortFun);
    // ROS_INFO("Find point in vector %d", min_distance_data_.size());
    for (size_t i = 0; i < min_distance_data_.size(); i++)
    {
      if (isPointInMatrix(min_distance_data_[i], find_point_in_.x, find_point_in_.y))
      {
        laneID_ = min_distance_data_[i].laneID;
        find_point_in_.heading = min_distance_data_[i].ref_data.theta;
        find_point_in_.laneID = min_distance_data_[i].laneID;
        find_point_in_.max_speed = min_distance_data_[i].ref_data.max_speed;
        // ROS_INFO("Find point in %d", laneID_);
        ROS_INFO("Find point is (%lf,%lf) @ %d heading:%lf speed:%lf)", find_point_in_.x, find_point_in_.y,
                 find_point_in_.laneID, find_point_in_.heading, find_point_in_.max_speed);
        return laneID_;
      }
    }
  }
  ROS_INFO("Find point over min distance vector");
#endif

  return laneID_;
}

// 计算 |p1 p2| X |p1 p|
float RefSender::getCross(const Point &p1, const Point &p2, const Point &p)
{
  return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

bool RefSender::isPointInMatrix(const MinRefPoint &rfd, const double &x_, const double &y_)
{
  Point p1, p2, p3, p4, p;
  double d_x = INTERVAL_X;  // 0.5米
  double d_y1 = rfd.ref_data.left_lane_width;
  double d_y2 = rfd.ref_data.right_lane_width;
  double radian = rfd.ref_data.theta;
  p.x = x_;
  p.y = y_;
  //第三像限
  p3.x = rfd.ref_data.point_x + d_x * cos(radian) - d_y1 * sin(radian);
  p3.y = rfd.ref_data.point_y + d_x * sin(radian) + d_y1 * cos(radian);
  //第二像限
  p2.x = rfd.ref_data.point_x + d_x / 2 * cos(radian) + d_y1 * sin(radian);
  p2.y = rfd.ref_data.point_y + d_x / 2 * sin(radian) - d_y1 * cos(radian);
  //第一像限
  p1.x = rfd.ref_data.point_x - d_x * cos(radian) + d_y2 * sin(radian);
  p1.y = rfd.ref_data.point_y - d_x * sin(radian) - d_y2 * cos(radian);
  //第四像限
  p4.x = rfd.ref_data.point_x - d_x * cos(radian) - d_y2 * sin(radian);
  p4.y = rfd.ref_data.point_y - d_x * sin(radian) + d_y2 * cos(radian);

  // ROS_INFO("1:(%lf,%lf) 2:(%lf,%lf) 3:(%lf,%lf) 4:(%lf,%lf)", p1.x, p1.y,
  // p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

  return getCross(p1, p2, p) * getCross(p3, p4, p) >= 0 && getCross(p2, p3, p) * getCross(p4, p1, p) >= 0;
  // return false;
}

int RefSender::mathDistance(const double &x_, const double &y_,
                            const std::vector<std::vector<double> > &ref_line_point_, MinRefPoint &min_rfd)
{
  double r_min_distance = MAX_DISTANCE_SQUARE;  // 12m以内
  int ret_ = -1;

  for (size_t i = 0; i < ref_line_point_.size(); i++)
  {
    RefLaneData rfd;
    //    vectorToRefLaneData(ref_line_point_[i], rfd);
    rfd.laneID = ref_line_point_[i][0];
    rfd.point_x = ref_line_point_[i][1] * GRID_SIZE + OFFSET_X;
    rfd.point_y = ref_line_point_[i][2] * GRID_SIZE + OFFSET_Y;
    rfd.d_from_begin = ref_line_point_[i][3] * GRID_SIZE;
    rfd.theta = ref_line_point_[i][4];
    rfd.kappa = ref_line_point_[i][5];
    rfd.dappa = ref_line_point_[i][6];
    rfd.left_lane_width = ref_line_point_[i][7];
    rfd.right_lane_width = ref_line_point_[i][8];

    double m_dis = pointDistanceSquare(x_, y_, rfd.point_x, rfd.point_y);
    if (r_min_distance > m_dis)
    {
      r_min_distance = m_dis;
      min_rfd.min_distance = m_dis;
      min_rfd.ref_data = rfd;
      min_rfd.laneID = rfd.laneID;
      ret_ = i;
      // ROS_INFO("LaneId:%d - %d, (%lf,%lf)->(%lf,%lf) min_d:%lf", rfd.laneID,
      // i, x_, y_, rfd.point_x, rfd.point_y,
      //          m_dis);
      // ROS_INFO("rfd:%d %lf %lf %lf %lf %lf %lf %lf %lf", rfd.laneID,
      // rfd.point_x, rfd.point_y, rfd.d_from_begin,
      //          rfd.theta, rfd.kappa, rfd.dappa, rfd.left_lane_width,
      //          rfd.right_lane_width);
    }
  }
  return ret_;
}

void RefSender::vectorToRefLaneData(const std::vector<double> &ref_lp_data_, RefLaneData &rfd_, double &ss_)
{
  rfd_.laneID = ref_lp_data_[0];
#if ENABLE_SZ_MAP  // GPS 坐标转港区坐标
  Vector3d llh_;
  Vector3d henu_;

  llh_(0) = ref_lp_data_[2] / SCALE + Y_lan0;
  llh_(1) = ref_lp_data_[1] / SCALE + X_lon0;
  llh_(2) = H_h0;

  transFusionLLHtoHarbourENU(llh_, henu_);

  rfd_.point_x = henu_[1];
  rfd_.point_y = henu_[0];
  rfd_.d_from_begin = ss_ + ref_lp_data_[3];

#else
  rfd_.point_x = ref_lp_data_[1] * GRID_SIZE + OFFSET_X;
  rfd_.point_y = ref_lp_data_[2] * GRID_SIZE + OFFSET_Y;
  rfd_.d_from_begin = ss_ + ref_lp_data_[3] * GRID_SIZE;
#endif

  rfd_.theta = ref_lp_data_[4];
  rfd_.kappa = ref_lp_data_[5];
  rfd_.dappa = ref_lp_data_[6];
  rfd_.left_lane_width = ref_lp_data_[7];
  rfd_.right_lane_width = ref_lp_data_[8];

  // ROS_INFO("s_sum:%lf now:%lf now-raw:%lf", ss_, ref_lp_data_[3] * GRID_SIZE,
  // ref_lp_data_[3]);
}

double RefSender::pointDistanceSquare(const double &x1, const double &y1, const double &x2, const double &y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

int RefSender::findLaneRefFromMap(const int &l_id_, std::vector<RefLaneData> &route_rfd, double &sum_s_)
{
  std::map<int, std::vector<RefPoints> >::iterator iter = ref_points_map.find(l_id_);
  if (iter == ref_points_map.end())
  {
    ROS_INFO("can not find %d lane", l_id_);
    return -1;
  }
  else
  {
    for (std::vector<RefPoints>::iterator it_loop = iter->second.begin(); it_loop != iter->second.end(); it_loop++)
    {
      RefLaneData rfd;
      translateRefPointsToRefLaneData(*it_loop, rfd, sum_s_);
      route_rfd.push_back(rfd);
    }
    sum_s_ = route_rfd.rbegin()->d_from_begin;
    ROS_INFO("ss:%lf route_rfd.end():%lf heading:%lf -> %lf", sum_s_, route_rfd.rbegin()->d_from_begin,
             route_rfd.rbegin()->theta, route_rfd.rbegin()->theta * 180 / M_PI);
    return iter->second.size();
  }
}

void RefSender::translateRefPointsToRefLaneData(RefPoints &rp_, RefLaneData &rfd_, double &ss_)
{
  rfd_.laneID = rp_.ref_line_id;

  rfd_.point_x = rp_.point.x - ZJ_NEW_MAP_X;
  rfd_.point_y = rp_.point.y - ZJ_NEW_MAP_Y;
  rfd_.d_from_begin = ss_ + rp_.d_from_line_begin;
  rfd_.kappa = rp_.kappa;
  rfd_.dappa = rp_.dappa;
  rfd_.theta = rp_.theta;
  rfd_.left_lane_width = rp_.line_width.at(0).left;
  rfd_.right_lane_width = rp_.line_width.at(0).right;
  for (std::vector<LineWidth>::iterator loop_i = rp_.line_width.begin(); loop_i != rp_.line_width.end(); loop_i++)
  {
    LineWidth line_width_temp;
    line_width_temp.left = loop_i->left;
    line_width_temp.right = loop_i->right;
    rfd_.line_width.push_back(line_width_temp);
  }
}

int RefSender::findLaneRefFromCSV(const int &l_id_, std::vector<RefLaneData> &route_rfd, double &sum_s_)
{
  std::map<int, std::vector<std::vector<double> > >::iterator iter;
  iter = ref_line_data.find(l_id_);
  if (iter == ref_line_data.end())
  {
    ROS_INFO("can not find %d lane", l_id_);
    return -1;
  }
  else
  {
    for (size_t i = 0; i < iter->second.size(); i++)
    {
      RefLaneData rfd;
      vectorToRefLaneData(iter->second[i], rfd, sum_s_);
      route_rfd.push_back(rfd);
    }
    sum_s_ = route_rfd.rbegin()->d_from_begin;
    ROS_INFO("ss:%lf route_rfd.end():%lf", sum_s_, route_rfd.rbegin()->d_from_begin);
    return iter->second.size();
  }
}

int RefSender::getPointFromRoute(const RouteData &rd_, const TaskPoint &vlp_, const vector<RefLaneData> &trrd_,
                                 vector<RefLaneData> &trrd_t_, int &last_, int &now_, int &next_, const int &last_ret_)
{
  ROS_INFO("goto getPointFromRoute %d  last:%d %d now:%d %d next:%d %d", (int)trrd_.size(), last_, rd_.data[last_],
           now_, rd_.data[now_], next_, rd_.data[next_]);
  int begin_loop_num = 0;
  int end_loop_num = 0;
  double r_min_distance = 10000000;  // 12m以内
  int ret_ = 0;
  int end_index = trrd_.size();

  int last_t_ = 0;
  int next_t_ = 0;

  if (now_ == 0)
  {
    last_t_ = 0;
    next_t_ = now_ + 1;
  }
  else if (now_ == (rd_.data.size() - 1))
  {
    last_t_ = now_ - 1;
    next_t_ = 1;
  }
  else
  {
    last_t_ = now_ - 1;
    next_t_ = now_ + 1;
  }

  if (last_ret_ == 0)
  {
    for (size_t i = 1; i < end_index; i++)
    {
      if (trrd_[i].laneID == rd_.data[now_] && trrd_[i - 1].laneID == rd_.data[last_t_])
      {
        begin_loop_num = i;
        end_loop_num = min((begin_loop_num + 1000), end_index);
        break;
      }
    }
  }
  else
  {
    for (size_t i = last_ret_; i > 1; i--)
    {
      if (trrd_[i].laneID == rd_.data[now_] && trrd_[i - 1].laneID == rd_.data[last_t_])
      {
        begin_loop_num = i;
        break;
      }
      else
      {
        begin_loop_num = last_ret_;
      }
    }

    for (size_t i = last_ret_; i < end_index; i++)
    {
      if (trrd_[i].laneID == rd_.data[now_] && trrd_[i + 1].laneID == rd_.data[next_t_])
      {
        end_loop_num = i;
        break;
      }
    }
  }

  ROS_INFO("last_ret %d begin_loop_num = %d, end_loop_num = %d", last_ret_, begin_loop_num, end_loop_num);

  if (end_loop_num > begin_loop_num && end_loop_num != 0)
  {
    for (size_t i = begin_loop_num; i < end_loop_num; i++)
    {
      double m_dis = pointDistanceSquare(vlp_.x, vlp_.y, trrd_[i].point_x, trrd_[i].point_y);
      if (r_min_distance > m_dis)
      {
        r_min_distance = m_dis;
        ret_ = i;
      }
    }
    ROS_INFO("Find the closet point is %d (%f,%f) at lane %d", ret_, trrd_[ret_].point_x, trrd_[ret_].point_y,
             trrd_[ret_].laneID);

    if (ret_ < last_ret_)
    {
      ret_ = last_ret_;
    }

    if (ret_ > 0)
    {
      for (size_t i = max(ret_ - 200, 0); i < min(ret_ + 600, end_index) + 1; i++)
      {
        RefLaneData rfd_;
        rfd_ = trrd_[i];
        trrd_t_.push_back(rfd_);
        i = i + 3;
      }
      ROS_INFO("get new route data %d from route size %d", (int)trrd_t_.size(), (int)trrd_.size());

      return ret_;
    }
    else
    {
      ROS_INFO("get new route data error");
      ret_ = last_ret_;
      return ret_;
    }
  }
  else
  {
    ROS_INFO("error last:%d now:%d next:%d", last_, now_, next_);
    ret_ = last_ret_;
    return ret_;
  }
}

void RefSender::findLaneIDFromRoute(const RouteData &rd_, const TaskPoint &tp_, const int spt_, int &last_, int &now_,
                                    int &next_)
{
  if (last_ == 0 && now_ == 0 && next_ == 0)
  {
    for (size_t loop_i = 0; loop_i < rd_.data.size(); loop_i++)
    {
      if (tp_.laneID == rd_.data[loop_i])
      {
        now_ = loop_i;
        break;
      }
    }
  }
  else
  {
    if (tp_.laneID == rd_.data[next_])
    {
      now_ = next_;
    }
  }

  ROS_INFO("tp_ %d now_ %d", tp_.laneID, now_);
  last_ = now_ - 1;
  if ((now_ - 1) < 0)
  {
    last_ = now_;
  }
  next_ = now_ + 1;
  if ((now_ + 1) > rd_.data.size())
  {
    next_ = now_;
  }
  ROS_INFO("tp_ %d last:%d %d now:%d %d next:%d %d", tp_.laneID, last_, rd_.data[last_], now_, rd_.data[now_], next_,
           rd_.data[next_]);
}

void RefSender::taskPointFind()
{
  ROS_INFO("Finding task (%lf,%lf) in map", task_click_.x, task_click_.y);
  task_click_recv_tip = 0;
  int find_mode = 1;
  int task_ret = findLaneIDWithXY(task_click_, find_mode);
  if (task_ret > 0)
  {
    task_click_.laneID = task_ret;
    ROS_INFO("Find task at Lane ID:%d", task_ret);
    taskPointPub(task_click_);
  }
}

void RefSender::vcuPointFind()
{
  ROS_INFO("Finding vcu  (%lf,%lf) heading %lf in map", vcu_location_.x, vcu_location_.y, vcu_location_.heading);
  vcu_location_recv_tip = 0;
  find_vcu_laneID_tip = 1;
  int find_mode = 0;
  int ret = findLaneIDWithXY(vcu_location_, find_mode);
  if (ret > 0)
  {
    vcu_location_.laneID = ret;
    ROS_INFO("Find vcu_location at Lane ID:%d", ret);
    vcuLocationPub();
#if TESTMODE1
    //局部路径发布测试//寻找当前点+道路id
    if (send_planing_ == 1)
    {
      ROS_INFO("get route %d point %d", route_data_.data.size(), task_route_ref_data.size());
      findLaneIDFromRoute(route_data_, vcu_location_, send_planing_first_, last_id_, now_id_, next_id_);

      vector<RefLaneData> task_route_ref_data_test_;
      route_last_ret_ = getPointFromRoute(route_data_, vcu_location_, task_route_ref_data, task_route_ref_data_test_,
                                          last_id_, now_id_, next_id_, route_last_ret_);
      ROS_INFO("goto Pub %d route", task_route_ref_data_test_.size());
      if (task_route_ref_data_test_.size() > 0)
      {
        routeRefPlanningPub(task_route_ref_data_test_);
        ROS_INFO("Pub now route");
      }
      else
      {
        ROS_INFO("get route error");
      }
    }
#endif  //局部路径发布测试

#if TESTMODE2
    //判断任务结束，重启任务
    if (send_planing_ == 1)
    {
      if (now_id_ == (route_data_.data.size() - 1))
      {
        last_id_ = 0;
        next_id_ = 0;
        now_id_ = 0;
        route_last_ret_ = 0;
      }
    }
#endif
  }
  else
  {
    ROS_INFO("Can not Find Lane ID");
  }
  find_vcu_laneID_tip = 0;
}

void RefSender::addExtraVirtualRefLaneData(RefLaneData &rfd_in_, RefLaneData &rfd_, double &begin_ss, double &add_ss_,
                                           int index_)
{
  double add_x = add_ss_ * index_ * sin(rfd_in_.theta);
  double add_y = add_ss_ * index_ * cos(rfd_in_.theta);

  rfd_.laneID = rfd_in_.laneID;
  rfd_.point_x = rfd_in_.point_x + add_x;
  rfd_.point_y = rfd_in_.point_y + add_y;
  rfd_.d_from_begin = begin_ss + add_ss_ * index_;
  rfd_.kappa = rfd_in_.kappa;
  rfd_.dappa = rfd_in_.dappa;
  rfd_.theta = rfd_in_.theta;
  rfd_.left_lane_width = rfd_in_.left_lane_width;
  rfd_.right_lane_width = rfd_in_.right_lane_width;
  rfd_.line_width = rfd_in_.line_width;
}

int RefSender::addBeforeBeginRef(const int &l_id_, std::vector<RefLaneData> &route_rfd, double &sum_s_)
{
  std::map<int, std::vector<RefPoints> >::iterator iter = ref_points_map.find(l_id_);
  if (iter == ref_points_map.end())
  {
    ROS_INFO("can not find %d lane", l_id_);
    return -1;
  }
  else
  {
    // std::vector< RefPoints >::iterator it_loop = iter->second.begin();
    RefLaneData base_rfd;
    translateRefPointsToRefLaneData(*iter->second.begin(), base_rfd, sum_s_);
    ROS_WARN("First Id %d %lf %lf", base_rfd.laneID, base_rfd.point_x, base_rfd.point_y);
    int add_lane_s = 60;
    double delta_lane_s = 0.5;
    for (size_t i = 0; i < add_lane_s; i++)
    {
      RefLaneData rfd_temp;
      int add_index_ = i - add_lane_s;
      addExtraVirtualRefLaneData(base_rfd, rfd_temp, sum_s_, delta_lane_s, add_index_);
      route_rfd.push_back(rfd_temp);
      ROS_WARN("Lane Id %d %lf %lf", rfd_temp.laneID, rfd_temp.point_x, rfd_temp.point_y);
    }
    return add_lane_s;
  }
}

int RefSender::addAfterEndRef(const int &l_id_, std::vector<RefLaneData> &route_rfd, double &sum_s_)
{
  std::map<int, std::vector<RefPoints> >::iterator iter = ref_points_map.find(l_id_);
  if (iter == ref_points_map.end())
  {
    ROS_INFO("can not find %d lane", l_id_);
    return -1;
  }
  else
  {
    // std::vector< RefPoints > &rfp_temp         = iter->second;
    // std::vector< RefPoints >::iterator it_loop = rfp_temp.rbegin();
    RefLaneData base_rfd;
    translateRefPointsToRefLaneData(*iter->second.rbegin(), base_rfd, sum_s_);
    ROS_WARN("Last Id %d %lf %lf", base_rfd.laneID, base_rfd.point_x, base_rfd.point_y);
    int add_lane_s = 60;
    double delta_lane_s = 0.5;
    for (size_t i = 0; i < add_lane_s; i++)
    {
      RefLaneData rfd_temp;
      int add_index_ = i;
      addExtraVirtualRefLaneData(base_rfd, rfd_temp, sum_s_, delta_lane_s, add_index_);
      route_rfd.push_back(rfd_temp);
      ROS_WARN("Lane Id %d %lf %lf", rfd_temp.laneID, rfd_temp.point_x, rfd_temp.point_y);
    }
    return add_lane_s;
  }
}

void RefSender::routePointFind()
{
  ROS_INFO("Finding %d Lane's Ref", route_data_.data_length);
  route_recv_tip = 0;
  find_laneID_ref_tip = 1;
  int get_route_tip = 1;
  task_route_ref_data.clear();
  int ret_sum = 0;
  double s_from_b = 0;

  int route_length = static_cast<int>(route_data_.data.size());
  int route_begin_ = *route_data_.data.begin();
  int route_end_ = *route_data_.data.rbegin();
  if (route_length == 2 && route_begin_ == route_end_)
  {
    // target and agv is during same lane
    //判断agv和目标点在道路的先后顺序
    int data_temp_ = route_begin_;
    int fret = 0;
    ROS_WARN("Target and AGV is during same lane %d", data_temp_);

    if (vcu_location_.d_from_line_begin > task_click_.d_from_line_begin)
    {
      get_route_tip = 0;
      ROS_ERROR("Target %lf is before AGV %lf is during lane %d", vcu_location_.d_from_line_begin,
                task_click_.d_from_line_begin, data_temp_);
    }
    else
    {
      fret = addBeforeBeginRef(route_begin_, task_route_ref_data, s_from_b);
      if (fret > 0)
      {
        ret_sum = ret_sum + fret;
        ROS_INFO("Extra Begin laneID : %d Ref point is :%d,total %d, sum %lf", route_begin_, fret, ret_sum, s_from_b);
      }
      fret = findLaneRefFromMap(data_temp_, task_route_ref_data, s_from_b);
      if (fret > 0)
      {
        ret_sum = ret_sum + fret;
        ROS_INFO("Finding laneID : %d Ref point is :%d,total %d, sum %lf", data_temp_, fret, ret_sum, s_from_b);
      }
      fret = addAfterEndRef(route_end_, task_route_ref_data, s_from_b);
      if (fret > 0)
      {
        ret_sum = ret_sum + fret;
        ROS_INFO("Extra End laneID : %d Ref point is :%d,total %d, sum %lf", route_begin_, fret, ret_sum, s_from_b);
      }
    }
  }
  else
  {
    int fret = 0;
    fret = addBeforeBeginRef(route_begin_, task_route_ref_data, s_from_b);
    if (fret > 0)
    {
      ret_sum = ret_sum + fret;
      ROS_INFO("Extra Begin laneID : %d Ref point is :%d,total %d, sum %lf", route_begin_, fret, ret_sum, s_from_b);
    }

    for (size_t loop_i = 0; loop_i < route_data_.data_length; loop_i++)
    {
      int data_temp_ = route_data_.data[loop_i];

#if ENABLE_ZJ_NEW_MAP

      fret = findLaneRefFromMap(data_temp_, task_route_ref_data, s_from_b);

#else

      fret = findLaneRefFromCSV(data_temp_, task_route_ref_data, s_from_b);

#endif
      if (fret > 0)
      {
        ret_sum = ret_sum + fret;
        ROS_INFO("Finding laneID : %d Ref point is :%d,total %d, sum %lf", data_temp_, fret, ret_sum, s_from_b);
      }
    }
    fret = addAfterEndRef(route_end_, task_route_ref_data, s_from_b);
    if (fret > 0)
    {
      ret_sum = ret_sum + fret;
      ROS_INFO("Extra End laneID : %d Ref point is :%d,total %d, sum %lf", route_begin_, fret, ret_sum, s_from_b);
    }
  }
  if (get_route_tip > 0)
  {
    routeRefPlanningPub(task_route_ref_data);
    routeDecisionInfoPub(task_route_ref_data);
  }

#if TESTMODE1
  //局部路径发布测试
  send_planing_ = 1;
  last_id_ = 0;
  next_id_ = 0;
  now_id_ = 0;
  route_last_ret_ = 0;
#endif  //局部路径发布测试
  // rvizAllPointPub();
  // rvizRouteRefPointPub(task_route_ref_data);
  find_laneID_ref_tip = 0;
  // task_route_ref_data.clear();
  // route_data_.data.clear();
  int test_d_ = task_route_ref_data.size();
  ROS_INFO("Tip Status route recv: %d  Find ref: %d  vector size: %d", route_recv_tip, find_laneID_ref_tip, test_d_);
}

void RefSender::hmiReset()
{
  hmi_cmd_type_ = 0;
  hmi_cmd_type_recv_tip = 0;
}

void RefSender::refSender()
{
  ros::Rate loop_rate(100);
  int main_loop_ = 0;
  ROS_INFO("vcu_locatio:%d csv:%d", vcu_location_recv_tip, get_csv_tip);
  while (ros::ok())
  {
    ++main_loop_;
    main_loop_ = main_loop_ % 10;

    //收到任务点
    if (task_click_recv_tip == 1 && get_csv_tip == 1)
    {
      taskPointFind();
    }

    if (hmi_cmd_type_recv_tip == 1)
    {
      taskPointPub(task_click_);
      hmiReset();
    }

    if (vcu_location_recv_tip == 1 && get_csv_tip == 1 && main_loop_ == 2)
    {
      vcuPointFind();
      // int size = vcu_location_vec.size();
      // if (size % 100 == 1 && size > 2000)
      // {
      // rvizVCUPointPubNewMap();
      rvizVcuPointPubTest();
      //   vcu_location_vec.clear();
      // }
    }

    if (route_recv_tip == 1 && get_csv_tip == 1)
    {
      routePointFind();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

}  // namespace map
}  // namespace superg_agv

using namespace superg_agv::map;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ref_sender");
  ros::NodeHandle n;

  double point_x = 0.0;
  double point_y = 0.0;
  if (argc == 3)
  {
    point_x = atof(argv[1]);
    point_y = atof(argv[2]);
  }
  ROS_INFO("point (%lf, %lf)", point_x, point_y);
  ROS_INFO("Goto Ref Sender.");
  RefSender ref_sender_node(n, point_x, point_y, argc);
  //主程序休眠
  ros::spin();
  return 0;
}