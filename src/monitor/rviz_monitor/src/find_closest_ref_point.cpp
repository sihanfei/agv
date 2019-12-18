#include "find_closest_ref_point.h" //我们自定义类（class）的头文件
//#include "glog_helper.h"

namespace superg_agv
{
namespace monitor
{

// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
FindClosestRefPointClass::FindClosestRefPointClass(string file_name) // 构造函数
{
  ROS_INFO("in class constructor of FindClosestRefPointClass");

  // GLogHelper gh(( char * )"FindClosestRefPointClass");
  //  SUPERG_INFO << "in class constructor of FindClosestRefPointClass";

  readRefFromCSV(file_name);
  isDataOk_ = initializeReferenceLineData();

  csv_file_num = 13;
  // val_to_remember_ = 0.0; //初始化储存数据的变量的值
}

//以下是一个成员协助函数，读取参考线文件
bool FindClosestRefPointClass::initializeReferenceLineData()
{
  ROS_INFO("Initializing ReferenceLineData");

  mapdata m_data_temp;
  std::map< int, std::vector< std::vector< double > > >::iterator ref_iter;
  for (ref_iter = ref_line_data.begin(); ref_iter != ref_line_data.end(); ref_iter++)
  {
    for (size_t i = 0; i < ref_iter->second.size(); i++)
    {

      m_data_temp._x = ref_iter->second[i][1] * GRID_SIZE + OFFSET_X;
      m_data_temp._y = ref_iter->second[i][2] * GRID_SIZE + OFFSET_Y;
      i              = i + 4;
      map_ref_line_data_.emplace_back(m_data_temp);
    }
  }
  return true;
}

int FindClosestRefPointClass::split(char dst[][80], char *str, const char *spl)
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

double FindClosestRefPointClass::pointDistanceSquare(const double x_1_, const double y_1_, const double x_2_,
                                                     const double y_2_)
{
  const double dx = x_1_ - x_2_;
  const double dy = y_1_ - y_2_;
  return dx * dx + dy * dy;
}

int FindClosestRefPointClass::findClosestRefPoint(double in_x, double in_y, double &out_x, double &out_y)
{
  float min_value = 1000000; // 1km范围内
  double ans      = 100000;
  int loc_num     = map_ref_line_data_.size() + 1;
  // ROS_INFO("goto Finding %f %f\n", in_x, in_y);
  for (int i = 0; i < map_ref_line_data_.size(); i++)
  {
    ans = pointDistanceSquare(in_x, in_y, map_ref_line_data_[i]._x, map_ref_line_data_[i]._y);

    if (ans == 0)
    {
      min_value = ans;
      loc_num   = i;
      // break;
    }

    if (ans < min_value)
    {
      min_value = ans;
      loc_num   = i;
    }
  }

  if (loc_num > map_ref_line_data_.size())
  {
    out_x = in_x;
    out_y = in_y;
    // md_out = md_in;
    return 0;
  }
  else
  {
    out_x = map_ref_line_data_[loc_num]._x;
    out_y = map_ref_line_data_[loc_num]._y;
    return loc_num;
  }
  //    return loc_num;
}

void FindClosestRefPointClass::readRefFromCSV(string file_name)
{
  char *home_path          = getenv("HOME");
  char ref_line_name[1024] = {0};
  sprintf(ref_line_name, "%s%s%s", home_path, MAP_REF_POINT_CONNECT_PATH, file_name.c_str());
  // sprintf(ref_line_name, "%s%s%s", home_path, MAP_REF_POINT_CONNECT_PATH, REF_LINE_NAME);

  ROS_DEBUG("ref_line name:%s", ref_line_name);

  int ref_line_data_key;
  vector< vector< double > > ref_line_data_value;

  ifstream infile;
  infile.open(ref_line_name); //将文件流对象与文件连接起来
  assert(infile.is_open());   //若失败,则输出错误消息,并终止程序运行

  vector< double > line_read_double;
  int read_column = 0; //列
  int read_row    = 0; //行
  char read_c     = 0; //字符
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
        read_column         = 0;
        int end_double_temp = atof(read_buf_temp);
        line_read_double.emplace_back(end_double_temp);
        ref_line_data_value.emplace_back(line_read_double);
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
            ref_line_data.insert(pair< int, vector< vector< double > > >(ref_line_data_key, ref_line_data_value));
            ROS_DEBUG("map %d lane %d data %d", ref_line_data_key, ref_line_data_value.size(),
                      ref_line_data_value[0].size());
            ref_line_data_value.clear();
          }
        }
        ref_line_data_key = t_int_temp;
        line_read_double.emplace_back(t_int_temp);
      }
      else
      {
        double t_double_temp = atof(read_buf_temp);
        line_read_double.emplace_back(t_double_temp);
      }
      read_column++;
    }
  }
  ref_line_data.insert(pair< int, vector< vector< double > > >(ref_line_data_key, ref_line_data_value));
  ROS_DEBUG("map %d lane %d data %d", ref_line_data.size(), ref_line_data_value.size(), ref_line_data_value[0].size());
  ref_line_data_value.clear();

  infile.close();
  memset(read_buf_temp, 0, sizeof(read_buf_temp));
  line_read_double.clear();
  read_column = 0; //列
  read_row    = 0; //行
  read_c      = 0; //字符
  ROS_DEBUG("FindClosestRefPointClass read map data ok!");
  // get_csv_tip = 1;
}

int FindClosestRefPointClass::findLaneIDWithXY(const double &x_, const double &y_)
{
  int laneID_ = 0;
  std::vector< MinRefPoint > min_distance_data_;

  //  std::map<int, double> min_distance_map;
  //  std::map<int, RefLaneData> min_ref_lane_data_map;
  std::map< int, std::vector< std::vector< double > > >::iterator iter;
  for (iter = ref_line_data.begin(); iter != ref_line_data.end(); iter++)
  {
    MinRefPoint mdd_temp_;
    int point_ser = mathDistance(x_, y_, iter->second, mdd_temp_);

    if (point_ser > -1)
    {
      // ROS_INFO("Find the laneID %d and the closest point (%lf,%lf)", iter->first, mdd_temp_.ref_data.point_x,
      //          mdd_temp_.ref_data.point_y);
      // ROS_INFO("min_distance %lf  left:%lf right:%lf min:%lf", mdd_temp_.min_distance,
      //          mdd_temp_.ref_data.left_lane_width, mdd_temp_.ref_data.right_lane_width,
      //          min(mdd_temp_.ref_data.left_lane_width * mdd_temp_.ref_data.left_lane_width,
      //              mdd_temp_.ref_data.right_lane_width * mdd_temp_.ref_data.right_lane_width));
      min_distance_data_.emplace_back(mdd_temp_);
      // if (mdd_temp_.min_distance < min(mdd_temp_.ref_data.left_lane_width * mdd_temp_.ref_data.left_lane_width,
      //                                  mdd_temp_.ref_data.right_lane_width * mdd_temp_.ref_data.right_lane_width))
      // {
      //   laneID_ = iter->first;
      //   return laneID_;
      // }
    }
  }
  ROS_DEBUG("Find point over map");

  if (min_distance_data_.size() > 0)
  {
    //按照数值从小到大排序,然后查找
    std::sort(min_distance_data_.begin(), min_distance_data_.end(), sortFun);
    ROS_DEBUG("Find point in vector %d", min_distance_data_.size());
    for (size_t i = 0; i < min_distance_data_.size(); i++)
    {
      if (isPointInMatrix(min_distance_data_[i], x_, y_))
      {
        laneID_ = min_distance_data_[i].laneID;
        ROS_DEBUG("Find point in %d", laneID_);
        return laneID_;
      }
    }
  }
  ROS_DEBUG("Find point over min distance vector");
  return laneID_;
}

// 计算 |p1 p2| X |p1 p|
float FindClosestRefPointClass::getCross(const Point &p1, const Point &p2, const Point &p)
{
  return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

bool FindClosestRefPointClass::isPointInMatrix(const MinRefPoint &rfd, const double &x_, const double &y_)
{
  Point p1, p2, p3, p4, p;
  double d_x    = 0.5 / GRID_SIZE; // 0.5米
  double d_y1   = rfd.ref_data.left_lane_width;
  double d_y2   = rfd.ref_data.right_lane_width;
  double radian = rfd.ref_data.theta;
  p.x           = x_;
  p.y           = y_;
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

  //  ROS_INFO("1:(%lf,%lf) 2:(%lf,%lf) 3:(%lf,%lf) 4:(%lf,%lf)", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

  return getCross(p1, p2, p) * getCross(p3, p4, p) >= 0 && getCross(p2, p3, p) * getCross(p4, p1, p) >= 0;
  // return false;
}

int FindClosestRefPointClass::mathDistance(const double &x_, const double &y_,
                                           const std::vector< std::vector< double > > &ref_line_point_,
                                           MinRefPoint &min_rfd)
{
  double r_min_distance = MAX_DISTANCE_SQUARE; // 12m以内
  int ret_              = -1;

  for (size_t i = 0; i < ref_line_point_.size(); i++)
  {
    RefLaneData rfd;
    //    vectorToRefLaneData(ref_line_point_[i], rfd);
    rfd.laneID           = ref_line_point_[i][0];
    rfd.point_x          = ref_line_point_[i][1] * GRID_SIZE + OFFSET_X;
    rfd.point_y          = ref_line_point_[i][2] * GRID_SIZE + OFFSET_Y;
    rfd.d_from_begin     = ref_line_point_[i][3] * GRID_SIZE;
    rfd.theta            = ref_line_point_[i][4];
    rfd.kappa            = ref_line_point_[i][5];
    rfd.dappa            = ref_line_point_[i][6];
    rfd.left_lane_width  = ref_line_point_[i][7];
    rfd.right_lane_width = ref_line_point_[i][8];

    double m_dis = pointDistanceSquare(x_, y_, rfd.point_x, rfd.point_y);
    if (r_min_distance > m_dis)
    {
      r_min_distance       = m_dis;
      min_rfd.min_distance = m_dis;
      min_rfd.ref_data     = rfd;
      min_rfd.laneID       = rfd.laneID;
      ret_                 = i;
      // ROS_INFO("LaneId:%d - %d, (%lf,%lf)->(%lf,%lf) min_d:%lf", rfd.laneID, i, x_, y_, rfd.point_x, rfd.point_y,
      //          m_dis);
      // ROS_INFO("rfd:%d %lf %lf %lf %lf %lf %lf %lf %lf", rfd.laneID, rfd.point_x, rfd.point_y, rfd.d_from_begin,
      //          rfd.theta, rfd.kappa, rfd.dappa, rfd.left_lane_width, rfd.right_lane_width);
    }
  }
  return ret_;
}

//自定义排序函数
bool FindClosestRefPointClass::sortFun(const MinRefPoint &d1, const MinRefPoint &d2)
{
  return d1.min_distance < d2.min_distance; //升序排列
}

int FindClosestRefPointClass::findLaneRefFromCSV(const int &l_id_, std::vector< RefLaneData > &route_rfd)
{
  std::map< int, std::vector< std::vector< double > > >::iterator iter;
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
      vectorToRefLaneData(iter->second[i], rfd);
      route_rfd.emplace_back(rfd);
    }
    return iter->second.size();
  }
}
void FindClosestRefPointClass::vectorToRefLaneData(const std::vector< double > &ref_lp_data_, RefLaneData &rfd_)
{
  rfd_.laneID           = ref_lp_data_[0];
  rfd_.point_x          = ref_lp_data_[1] * GRID_SIZE + OFFSET_X;
  rfd_.point_y          = ref_lp_data_[2] * GRID_SIZE + OFFSET_Y;
  rfd_.d_from_begin     = ref_lp_data_[3] * GRID_SIZE;
  rfd_.theta            = ref_lp_data_[4];
  rfd_.kappa            = ref_lp_data_[5];
  rfd_.dappa            = ref_lp_data_[6];
  rfd_.left_lane_width  = ref_lp_data_[7];
  rfd_.right_lane_width = ref_lp_data_[8];
}

} // namespace superg_agv
} // namespace monitor