#ifndef GET_MAP_XY_COLOR_H
#define GET_MAP_XY_COLOR_H
//常用的成员
#include <sstream>
#include <stdio.h>
#include <vector>

#include "ros/ros.h" //惯例添加

// OpenCV2标准头文件
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

namespace superg_agv
{
namespace monitor
{
using namespace std;

#define ROAD_IMG_PATH "/work/superg_agv/src/monitor/rviz_monitor/data/map_data"
#define ROAD_IMG_NAME "/1.bmp"

#define MAP_OFFSET_X -496.90
#define MAP_OFFSET_Y -109.04
#define MAP_GRID_SIZE 0.253

class GetMapXYcolorClass
{
public:
  GetMapXYcolorClass();
  bool isDataOk_;
  bool getMapXYcolor(double in_x, double in_y);

private:
  // 私有的数据成员只能在该类中被调用

  // int mapdata_[1100][1100];
  vector< vector< int > > array_mapdata_; //定义二维数组

  int g_img_cols_ = 0;
  int g_img_rows_ = 0;

  bool initializeMapData();
};

} // namespace superg_agv
} // namespace monitor
#endif // GET_MAP_XY_COLOR_H