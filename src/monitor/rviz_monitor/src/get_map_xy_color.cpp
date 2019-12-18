#include "get_map_xy_color.h" //我们自定义类（class）的头文件

namespace superg_agv
{
namespace monitor
{
// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
GetMapXYcolorClass::GetMapXYcolorClass()
{
  ROS_INFO("in class constructor of RvizObstacleSimClass");
  isDataOk_ = initializeMapData();

  // val_to_remember_ = 0.0; //初始化储存数据的变量的值
}

bool GetMapXYcolorClass::initializeMapData()
{
  //获取地图路径
  char *home_path = getenv("HOME");
  //黑白地图路径

  char black_and_white_map_path[1024] = {0};
  sprintf(black_and_white_map_path, "%s%s%s", home_path, ROAD_IMG_PATH, ROAD_IMG_NAME);
  ROS_INFO("black and white map:%s\n", black_and_white_map_path);
  //读取黑白地图
  cv::Mat BW_map = cv::imread(black_and_white_map_path);
  if (BW_map.empty())
  {
    ROS_ERROR("open black and white map error\n");
    return false;
  }
  //分离通道
  std::vector< cv::Mat > channels_;
  cv::split(BW_map, channels_);
  ROS_INFO("BW_map channels=%d\n", BW_map.channels());
  g_img_cols_ = BW_map.cols;
  g_img_rows_ = BW_map.rows;
  ROS_INFO("BW_map.rows[%d] BW_map.cols[%d]\n", BW_map.rows, BW_map.cols);
  //获取地图道路范围
  // ofstream in;
  // in.open("com.txt", ios::trunc); // ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
  vector< int > v;        //定义一维数组
  array_mapdata_.clear(); //将二维数组清空
  for (size_t i = 0; i < BW_map.rows; i++)
  {
    v.clear(); //子数组返回时要清除
    for (size_t j = 0; j < BW_map.cols; j++)
    {
      if (channels_[0].at< uchar >(i, j) == 0 and channels_[1].at< uchar >(i, j) == 0 and
          channels_[2].at< uchar >(i, j) == 0)
      {
        v.push_back(1);
      }
      else
      {
        v.push_back(0);
      }
    }
    array_mapdata_.push_back(v);
  }
  ROS_INFO("mapdata_");
  return true;
}

bool GetMapXYcolorClass::getMapXYcolor(double in_x, double in_y)
{
  //根据X Y 转换 成图像对应的像素点坐标
  int img_y = ( int )((in_x - MAP_OFFSET_X) / MAP_GRID_SIZE);
  int img_x = g_img_rows_ - ( int )((in_y - MAP_OFFSET_Y) / MAP_GRID_SIZE);
  if (img_x >= 0 and img_x < g_img_cols_ and img_y >= 0 and img_y < g_img_rows_)
  {
    if (array_mapdata_[img_x][img_y] == 1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  // ROS_INFO("img_x=%d  img_y=%d\n", img_x, img_y);

  return false;
}

} // namespace superg_agv
} // namespace monitor