#include "rviz_show_map.h"

namespace superg_agv
{
namespace monitor
{
using namespace cv;

// 构造函数
RvizShowMapClass::RvizShowMapClass(ros::NodeHandle &nodehandle, std::string map_file_path,
                                   std::string map_img_file_name, double map_offset_x, double map_offset_y,
                                   double map_grid_size)
    : nh_(nodehandle)
{
  ROS_INFO("in class constructor of RvizShowMapClass");
  map_file_path_     = map_file_path;
  map_img_file_name_ = map_img_file_name;
  map_offset_x_      = map_offset_x;
  map_offset_y_      = map_offset_y;
  map_grid_size_     = map_grid_size;
  initializePublishers();
  isDataOk_ = initializeMapData();
}

bool RvizShowMapClass::initializeMapData()
{
  //获取地图路径
  char *home_path               = getenv("HOME");
  char map_date_path_name[1024] = {0};
  sprintf(map_date_path_name, "%s%s%s", home_path, map_file_path_.c_str(), map_img_file_name_.c_str());
  ROS_INFO("map date path name:%s\n", map_date_path_name);

  //读取图像
  cv::Mat matSrc = cv::imread(map_date_path_name); //,
  // CV_LOAD_IMAGE_COLOR);
  if (matSrc.empty())
  {
    ROS_INFO("open error\n");
    return false;
  }
  img_rows_ = matSrc.rows;
  img_cols_ = matSrc.cols;

  //分离通道
  std::vector< cv::Mat > channels;
  cv::split(matSrc, channels);
  ROS_INFO("matSrc channels = %d", matSrc.channels());

  // int map_data_len_ = matSrc.cols * matSrc.rows;

  //循环取每个像素点的BGR
  int icount = 0;
  for (int i = matSrc.rows - 1; i >= 0; i--)
  {
    //得到每行 B G R 的数据指针
    const uchar *curr_r_b = channels[0].ptr< const uchar >(i);
    const uchar *curr_r_g = channels[1].ptr< const uchar >(i);
    const uchar *curr_r_r = channels[2].ptr< const uchar >(i);
    uint8_t ucolor        = 120;
    for (int j = 0; j < matSrc.cols; j++)
    {

      if ((curr_r_b[j] >= 0 && curr_r_b[j] <= 0) && (curr_r_g[j] >= 0 && curr_r_g[j] <= 0) &&
          (curr_r_r[j] >= 0 && curr_r_r[j] <= 0))
      {

        // ROS_INFO("黑");
        // ucolor = 80;
        ucolor = 80;
      }
      else if ((curr_r_b[j] >= 255 && curr_r_b[j] <= 255) && (curr_r_g[j] >= 255 && curr_r_g[j] <= 255) &&
               (curr_r_r[j] >= 255 && curr_r_r[j] <= 255))
      {

        // ROS_INFO("白");
        ucolor = 0;
      }
      else if ((curr_r_b[j] >= 0 && curr_r_b[j] <= 0) && (curr_r_g[j] >= 0 && curr_r_g[j] <= 0) &&
               (curr_r_r[j] >= 0 && curr_r_r[j] <= 255))
      {

        // ROS_INFO("红");
        ucolor = 150;
      }
      else if ((curr_r_b[j] >= 11 && curr_r_b[j] <= 25) && (curr_r_g[j] >= 43 && curr_r_g[j] <= 255) &&
               (curr_r_r[j] >= 46 && curr_r_r[j] <= 255))
      {
        // ROS_INFO("橙");
        ucolor = 180;
      }
      else if ((curr_r_b[j] >= 26 && curr_r_b[j] <= 34) && (curr_r_g[j] >= 43 && curr_r_g[j] <= 255) &&
               (curr_r_r[j] >= 46 && curr_r_r[j] <= 255))
      {

        // ROS_INFO("黄");
        ucolor = 210;
      }
      else if ((curr_r_b[j] >= 0 && curr_r_b[j] <= 0) && (curr_r_g[j] >= 0 && curr_r_g[j] <= 255) &&
               (curr_r_r[j] >= 0 && curr_r_r[j] <= 0))
      {

        // ROS_INFO("绿");
        ucolor = 250;
      }
      else if ((curr_r_b[j] >= 0 && curr_r_b[j] <= 255) && (curr_r_g[j] >= 0 && curr_r_g[j] <= 0) &&
               (curr_r_r[j] >= 0 && curr_r_r[j] <= 0))
      {

        // ROS_INFO("蓝");
        // ucolor = 50;
        ucolor = 60;
      }
      else
      {

        // ROS_INFO("未知");
      }
      map_colors_.emplace_back(ucolor);
      icount++;
    }
  }
  return true;
}
void RvizShowMapClass::initializePublishers()
{
  showmap_pub_       = nh_.advertise< nav_msgs::OccupancyGrid >("/monitor/showmap", 1, true);
  container_sim_pub_ = nh_.advertise< visualization_msgs::MarkerArray >("/monitor/container_sim", 100);
}
void RvizShowMapClass::rvizMapPub()
{
  if (!isDataOk_)
    return;
  //集装箱marker
  visualization_msgs::MarkerArray container_sim_markerarray_;
  for (size_t i = 0; i < 12; i++)
  {
    visualization_msgs::Marker container_sim_marker_;
    container_sim_marker_.header.frame_id = "/odom";
    container_sim_marker_.header.stamp    = ros::Time::now();
    container_sim_marker_.ns              = "container_sim";
    container_sim_marker_.action          = visualization_msgs::Marker::ADD;
    // container_sim_marker_.pose.orientation.w = 1.0;
    container_sim_marker_.id      = i + 1;
    container_sim_marker_.type    = visualization_msgs::Marker::CUBE;
    container_sim_marker_.scale.x = 2.4;
    container_sim_marker_.scale.y = 12;
    container_sim_marker_.scale.z = 2.8;
    if (container_color_[i] == 1)
    {
      container_sim_marker_.color.r = 1.0;
      container_sim_marker_.color.g = 0.4;
      container_sim_marker_.color.b = 0.0;
      container_sim_marker_.color.a = 1.0;
    }
    else
    {
      container_sim_marker_.color.r = 0.0;
      container_sim_marker_.color.g = 0.6;
      container_sim_marker_.color.b = 1.0;
      container_sim_marker_.color.a = 1.0;
    }

    container_sim_marker_.pose.position.x  = container_x_[i];
    container_sim_marker_.pose.position.y  = container_y_[i];
    container_sim_marker_.pose.position.z  = 1.5;
    container_sim_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, container_angle_[i]);
    container_sim_markerarray_.markers.emplace_back(container_sim_marker_);
  }

  // container_sim_pub_.publish(container_sim_markerarray_);

  nav_msgs::OccupancyGrid msg;
  msg.header.seq      = 0;
  msg.header.frame_id = "/odom";
  msg.header.stamp    = ros::Time::now();

  msg.info.map_load_time        = msg.header.stamp;
  msg.info.resolution           = map_grid_size_; //地图珊格大小
  msg.info.width                = img_cols_;      //地图大小 取图像像素 1000*994
  msg.info.height               = img_rows_;
  msg.info.origin.position.x    = map_offset_x_; //地图偏移
  msg.info.origin.position.y    = map_offset_y_;
  msg.info.origin.position.z    = 0;
  msg.info.origin.orientation.x = 0;
  msg.info.origin.orientation.y = 0;
  msg.info.origin.orientation.z = 0;
  msg.info.origin.orientation.w = 0;
  msg.data.resize(img_cols_ * img_rows_);
  std::list< uint8_t >::iterator iter;
  int icount_ = 0;
  for (iter = map_colors_.begin(); iter != map_colors_.end(); iter++)
  {
    msg.data[icount_] = *iter;
    icount_++;
  }
  showmap_pub_.publish(msg);
}
} // namespace superg_agv
} // namespace monitor
