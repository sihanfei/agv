#ifndef RVIZ_SHOW_MAP_H
#define RVIZ_SHOW_MAP_H
//常用的成员
#include <list>
#include <sstream>
// OpenCV2标准头文件
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h" //惯例添加

#include "tf/tf.h"

//需要用到的消息类型
#include "nav_msgs/OccupancyGrid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace cv;

namespace superg_agv
{
namespace monitor
{

//#define MAP_DATA_PATH "/work/superg_agv/src/monitor/rviz_monitor/data/map_data/"
// #define MAP_DATA_NAME "/map1.bmp"

// #define MAP_OFFSET_X -260
// #define MAP_OFFSET_Y -70
// #define MAP_GRID_SIZE 0.115

//构造函数是在类里面使用的函数，构造函数的名称与类额名称相同
class RvizShowMapClass
{
public:
  // main函数需要一个ROS的节点句柄，并通过这个节点句柄连接到这个构造函数
  RvizShowMapClass(ros::NodeHandle &nodehandle, std::string map_file_path, std::string map_img_file_name,
                   double map_offset_x, double map_offset_y, double map_grid_size);
  bool isDataOk_;
  void rvizMapPub();

private:
  // 私有的数据成员只能在该类中被调用
  ros::NodeHandle nh_; // 通过这个节点句柄连接main函数和构造函数
  std::list< uint8_t > map_colors_;
  double container_x_[12] = {-11.024, -1.260, 2.542,   12.307, 1.019, 9.856,
                             25.624,  34.312, -11.031, -4.898, 5.588, 13.574}; // sin(40)= 0.64279*12.2=7.842
  double container_y_[12] = {30.506, 22.529, 47.156, 39.178, 58.085, 69.044,
                             88.246, 91.175, 67.893, 79.879, 88.265, 98.055}; // cos(40)= 0.76604*12.2=9.345
  double container_angle_[12] = {0.8901, 0.8901, 0.8901, 0.8901, 2.4609, 2.4609,
                                 2.4609, 0.8901, 2.4609, 2.4609, 2.4609, 2.4609};
  int container_color_[12] = {0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0};

  //定义变量用于在成员函数中传输数据
  int img_rows_;
  int img_cols_;

  std::string map_file_path_;
  std::string map_img_file_name_;
  double map_offset_x_;
  double map_offset_y_;
  double map_grid_size_;

  //这里变量后面都加了下划线，作用是提醒这变量只能在该类 中被调用
  ros::Publisher showmap_pub_;
  ros::Publisher container_sim_pub_;

  bool initializeMapData();
  void initializePublishers();
};
} // namespace monitor
} // namespace superg_agv

#endif // RVIZ_SHOW_MAP_H