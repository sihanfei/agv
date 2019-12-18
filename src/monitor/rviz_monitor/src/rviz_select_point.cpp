#include "rviz_select_point.h"

namespace superg_agv
{
namespace monitor
{
RvizSelectPointClass::RvizSelectPointClass(ros::NodeHandle &nodehandle, string file_name) : nh_(nodehandle) // 构造函数
{
  ROS_INFO("in class constructor of RvizSelectPointClass");
  get_ref_line.reset(new GetRefLine());

  std::string home_path           = getenv("HOME");
  std::string workplace_path      = "/work/superg_agv/src/routing/map_new/data";
  std::string ref_line_file_name  = home_path + workplace_path + "/ref_line.json";
  std::string ref_point_file_name = home_path + workplace_path + "/ref_points.json";

  get_ref_line->setRefLineInit();
  get_ref_line->getRefLineFromFile2Map(ref_line_file_name, ref_line_map);
  get_ref_line->getRefPointFromFile2Map(ref_point_file_name, ref_line_map, ref_points_map, ref_points_vec);

  // find_closest_ref_point_.reset(new FindClosestRefPointClass(file_name));
  initializeSubscribers(); // 需要通过成员协助函数帮助构建subscriber，在构造函数中做初始化的工作
  initializePublishers();

  object_point_x_ = 0.0;
  object_point_y_ = 0.0;

  // val_to_remember_ = 0.0; //初始化储存数据的变量的值
}

void RvizSelectPointClass::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  rviz_clicked_sub_ = nh_.subscribe("/clicked_point", 10, &RvizSelectPointClass::rvizClickedCallback, this);

  route_laneID_array_sub_ =
      nh_.subscribe("/map/route_laneID_arry", 10, &RvizSelectPointClass::recvRouteLaneIDArrayCallback, this);
}
void RvizSelectPointClass::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  select_point_info_pub_ = nh_.advertise< visualization_msgs::MarkerArray >("/monitor/select_point_info", 1, true);
  task_click_pub_        = nh_.advertise< geometry_msgs::PointStamped >("/monitor/rviz_click_lane", 1, true);

  rviz_route_ref_pub_ = nh_.advertise< visualization_msgs::MarkerArray >("/monitor/route_msg", 1, true);

  // vms_info_pub_ = nh_.advertise< common_msgs::CommunicationVMSINFO >("/common/vms_info", 10, true);
}

void RvizSelectPointClass::rvizClickedCallback(const geometry_msgs::PointStampedConstPtr &msg)
{
  // if (!find_closest_ref_point_->isDataOk_)
  // {
  //   ROS_ERROR("find_closest_ref_point_.isDataOk_ is false");
  //   return;
  // }
  // if (!get_map_xy_color_.isDataOk_)
  // {
  //   ROS_ERROR("get_map_xy_color_.isDataOk_ is false");
  //   return;
  // }
  // //判断点击的是否是道路
  // bool isroad_ = get_map_xy_color_.getMapXYcolor(msg->point.x, msg->point.y);

  //查询最近的参考线点
  double real_point_x_, real_point_y_;

  RefPoints ref_point_temp;
  ref_point_temp.point.x = msg->point.x;
  ref_point_temp.point.y = msg->point.y;
  ref_point_temp.point.z = 0.0;

  int laneID = get_ref_line->findXYZFromRefPointVecTraversal(ref_point_temp, ref_points_vec);

  real_point_x_ = ref_point_temp.point.x;
  real_point_y_ = ref_point_temp.point.y;
  //  find_closest_ref_point_->findClosestRefPoint(msg->point.x, msg->point.y, real_point_x_, real_point_y_);

  ROS_DEBUG("recieve_point_=[%f]-[%f],real_point_=[%f]-[%f]", msg->point.x, msg->point.y, real_point_x_, real_point_y_);
  ROS_DEBUG("laneID = [%d]", laneID);

  double point_x = real_point_x_;
  double point_y = real_point_y_;

  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom";
  marker.header.stamp    = ros::Time::now();
  marker.action          = visualization_msgs::Marker::ADD;
  marker.color.a         = 1;

  geometry_msgs::Pose pose;

  //鼠标点击的位置 如果在路上，就什么也不显示，如果不在路上，就显示NO WAY
  // 2019-08-29 不再判断选点是否在路上。直接查找最近的参考点
  marker.id      = 1;
  marker.ns      = "Mouse selection";
  marker.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1;

  pose.position.x    = msg->point.x;
  pose.position.y    = msg->point.y;
  pose.position.z    = 2;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  marker.color.b = 0;
  marker.color.g = 0;
  marker.color.r = 1;
  marker.text    = "ClickedPoint";

  // if (isroad_)
  // {
  //   marker.color.b = 0;
  //   marker.color.g = 1;
  //   marker.color.r = 0;
  //   marker.text    = "OK";
  // }
  // else
  // {
  //   marker.color.b = 0;
  //   marker.color.g = 0;
  //   marker.color.r = 1;
  //   marker.text    = "No way";
  // }
  marker.pose     = pose;
  marker.lifetime = ros::Duration();
  markerArray.markers.emplace_back(marker);

  //标志
  marker.ns      = "ref_point Mark";
  marker.type    = visualization_msgs::Marker::ARROW;
  marker.scale.x = 5.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.id       = 2;
  marker.color.b  = 0;
  marker.color.g  = 1;
  marker.color.r  = 0;
  pose.position.x = point_x;
  pose.position.y = point_y;
  pose.position.z = 5;

  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2, 0);

  marker.pose     = pose;
  marker.lifetime = ros::Duration();
  markerArray.markers.emplace_back(marker);

  //文本
  marker.id      = 3;
  marker.ns      = "ref_point Tips";
  marker.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.x = 0.0;
  marker.scale.y = 0.0;
  marker.scale.z = 1;

  marker.color.b     = 0;
  marker.color.g     = 0;
  marker.color.r     = 1;
  pose.position.x    = point_x;
  pose.position.y    = point_y;
  pose.position.z    = 6;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  marker.text     = "GO GO GO";
  marker.pose     = pose;
  marker.lifetime = ros::Duration();
  markerArray.markers.emplace_back(marker);

  // ROS_INFO("clicked_info_pub");
  select_point_info_pub_.publish(markerArray);

  //查询道路ID
  geometry_msgs::PointStamped pointStamped_;
  pointStamped_.header.stamp = ros::Time::now();
  // double map_x, map_y;
  // map_x = (point_x - OFFSET_X) / GRID_SIZE;
  // map_y = (point_y - OFFSET_Y) / GRID_SIZE;
  // ROS_INFO("MAP X_Y %f %f", map_x, map_y);
  // int laneID = find_closest_ref_point_->findLaneIDWithXY(point_x, point_y);

  pointStamped_.point.x = point_x;
  pointStamped_.point.y = point_y;
  pointStamped_.point.z = laneID;
  task_click_pub_.publish(pointStamped_);

  object_point_x_ = point_x;
  object_point_y_ = point_y;

  visualization_msgs::MarkerArray markerArray_;
  visualization_msgs::Marker ref_points, route_id, route_arrow;

  ref_points.header.frame_id = route_id.header.frame_id = route_arrow.header.frame_id = "/odom";
  ref_points.header.stamp = route_id.header.stamp = route_arrow.header.stamp = ros::Time::now();

  ref_points.ns  = "/route_plan_point";
  route_id.ns    = "/route_plan_laneid";
  route_arrow.ns = "/route_plan_arrow";

  ref_points.action = route_id.action = route_arrow.action = visualization_msgs::Marker::DELETEALL;
  markerArray_.markers.emplace_back(ref_points);
  markerArray_.markers.emplace_back(route_id);
  markerArray_.markers.emplace_back(route_arrow);
  rviz_route_ref_pub_.publish(markerArray_);
}

void RvizSelectPointClass::recvRouteLaneIDArrayCallback(const std_msgs::Int64MultiArrayConstPtr &msg)
{
  RouteData route_data_;

  route_data_.vcu_ID      = 40;
  route_data_.data_status = 1;
  route_data_.data_length = msg->layout.dim[0].size;

  route_data_.data.clear();
  ROS_INFO("RvizSelectPointClass lane num = %d ", route_data_.data_length);
  for (size_t loop_i = 0; loop_i < route_data_.data_length; loop_i++)
  {
    int data_temp_ = msg->data[loop_i];
    route_data_.data.emplace_back(data_temp_);
    // ROS_INFO("%d ", data_temp_);
  }
  visualization_msgs::MarkerArray markerArray;

  vector< RefPoints > task_route_ref_data;
  int ret_sum = 0;
  vector< int > v_;
  for (size_t loop_i = 0; loop_i < route_data_.data_length; loop_i++)
  {
    task_route_ref_data.clear();
    std::stringstream ss;

    int data_temp_ = route_data_.data[loop_i];
    int nRet       = std::count(v_.begin(), v_.end(), data_temp_);
    v_.emplace_back(data_temp_);

    ROS_DEBUG("route %d find %d count", data_temp_, nRet);

    int fret; // = find_closest_ref_point_->findLaneRefFromCSV(data_temp_, task_route_ref_data);

    std::map< int, std::vector< RefPoints > >::iterator iter; // ;
    iter = ref_points_map.find(data_temp_);
    if (iter != ref_points_map.end())
    {
      fret                = iter->second.size();
      task_route_ref_data = iter->second;
      // ROS_DEBUG_STREAM("Find, the value is " << iter->second);

      ret_sum = ret_sum + fret;
      ROS_DEBUG("Finding laneID : %d Ref point is :%d,total %d", data_temp_, fret, ret_sum);
      //每一条线路 发送一组marker
      visualization_msgs::Marker ref_points, route_id, route_arrow;
      ref_points.header.frame_id = route_id.header.frame_id = route_arrow.header.frame_id = "/odom";
      ref_points.header.stamp = route_id.header.stamp = route_arrow.header.stamp = ros::Time::now();

      ref_points.ns  = "/route_plan_point";
      route_id.ns    = "/route_plan_laneid";
      route_arrow.ns = "/route_plan_arrow";

      ref_points.action = route_id.action = route_arrow.action = visualization_msgs::Marker::ADD;
      ref_points.pose.orientation.w = route_id.pose.orientation.w = 1.0;
      ref_points.id = route_id.id = route_arrow.id = loop_i;

      ref_points.type  = visualization_msgs::Marker::POINTS;
      route_id.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
      route_arrow.type = visualization_msgs::Marker::ARROW;

      // POINTS markers use x and y scale for width/height respectively
      ref_points.scale.x = 1.0;
      ref_points.scale.y = 1.0;
      // TEXT markers use z
      route_id.scale.z = 1;

      route_arrow.scale.x = 5.0;
      route_arrow.scale.y = 0.1;
      route_arrow.scale.z = 0.2;

      // Points are green
      ref_points.color.r = 0.0;
      ref_points.color.g = 0.0;
      ref_points.color.b = 1.0;
      ref_points.color.a = 1.0;
      // Points are red
      route_id.color.r = 0.0;
      route_id.color.g = 0.0;
      route_id.color.b = 0.0;
      route_id.color.a = 1.0;
      // Points are
      route_arrow.color.r = 1.0;
      route_arrow.color.g = 1.0;
      route_arrow.color.b = 0.0;
      route_arrow.color.a = 1.0;

      for (size_t i = 0; i < task_route_ref_data.size(); i++)
      {
        geometry_msgs::Point p;
        p.x = task_route_ref_data[i].point.x;
        p.y = task_route_ref_data[i].point.y;
        p.z = 0.3;
        ref_points.points.emplace_back(p);
        if (i == 0)
        {
          route_arrow.pose.position.x  = task_route_ref_data[i].point.x;
          route_arrow.pose.position.y  = task_route_ref_data[i].point.y;
          route_arrow.pose.position.z  = 5;
          route_arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 2, 0);

          ss << loop_i << "-" << data_temp_;
          route_id.text = ss.str();
          ss.str("");
          route_id.pose.position.x = task_route_ref_data[i].point.x;
          route_id.pose.position.y = task_route_ref_data[i].point.y;
          route_id.pose.position.z = 5 + nRet;
        }
        i = i + 3;
      }
      markerArray.markers.emplace_back(ref_points);
      markerArray.markers.emplace_back(route_arrow);
      markerArray.markers.emplace_back(route_id);

    } // end if(fret>0)
  }

  rviz_route_ref_pub_.publish(markerArray);

  // //发送行走命令
  // common_msgs::CommunicationVMSINFO cmd_msg_;
  // cmd_msg_.header.stamp = ros::Time::now();
  // common_msgs::TargetPoint target_point_;
  // cmd_msg_.CMD_Type.data       = 1;
  // target_point_.x.data         = int(object_point_x_ * 1000);
  // target_point_.y.data         = int(object_point_x_ * 1000);
  // target_point_.heading.data   = 0;
  // target_point_.max_speed.data = 10;
  // cmd_msg_.target_positions.emplace_back(target_point_);
  // vms_info_pub_.publish(cmd_msg_);
}

} // namespace superg_agv
} // namespace monitor