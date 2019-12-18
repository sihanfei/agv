#ifndef MAP_INCLUDE_FIND_REF_LINE_H_
#define MAP_INCLUDE_FIND_REF_LINE_H_

#include "map_common_utils.h"
#include "ros/ros.h"
#include <algorithm>
#include <iostream> // std::cout
#include <map>
#include <string>
#include <vector>

using namespace std;

namespace superg_agv
{
namespace map
{
class FindRefLine
{
public:
  FindRefLine() {}
  virtual ~FindRefLine() {}

  RefPoints findXYZFromRefPointVecMethod(const RefPoints point_in,
                                         const std::vector< RefPoints > &ref_points_vec_caches)
  {
    RefPoints point_out;
    std::vector< RefPoints > find_points_caches;
    std::vector< RefPoints >::iterator it_b    = ref_points_vec.begin();
    std::vector< RefPoints >::iterator it_e    = ref_points_vec.end();
    std::vector< RefPoints >::iterator it_loop = it_b;

    cout << "Will go to find (" << point_in.point.x << ", " << point_in.point.y << ", " << point_in.point.z << " )"
         << endl;
    int loop_index = 0;
    while (it_loop != it_e)
    {
      it_b    = it_loop;
      it_loop = std::find_if(it_b, it_e, PointFindFun(point_in));
      cout << "We had find " < < loop_index < < "answer: (" << it_loop.point.x << ", " << it_loop.point.y << ", "
                                                            << it_loop.point.z << " )" << endl;
      find_points_caches.push_back(it_loop);
      ++it_loop;
      ++loop_index;
    }

    if (!find_points_caches.empty()) //结果不为空
    {
      //排序后 遍历该数组找到
    }
    else //结果为空
    {
      // point_out 赋空
    }

    return point_out;
  }

  int findXYZFromRefPointVec(TaskPoint3D point_in)
  {
    RefPoints ref_point_temp;
    ref_point_temp.point.x = point_in.x;
    ref_point_temp.point.y = point_in.y;
    ref_point_temp.point.z = point_in.z;
    ref_point_temp.theta   = point_in.heading;

    std::vector< RefPoints >::iterator it;
    // it = std::find_if(ref_points_vec.begin(), ref_points_vec.end(), IsOdd);
    //判断参考线类型、对直线判断，圆弧进行判断，选择两端点计算高及三边差值
    return 1;
  }

public:
  struct PointFindFun
  {
    PointFindFun(RefPoints task_point) : aim_point(task_point) {}
    bool operator()(const std::vector< RefPoints >::value_type &point_caches_)
    {
      double distance_ = (aim_point.point.x - point_caches_.point.x) * (aim_point.point.x - point_caches_.point.x) +
                         (aim_point.point.x - point_caches_.point.x) * (aim_point.point.y - point_caches_.point.y);
      double width_ = 0.0;
      for (size_t i = 0; i < point_caches_.line_count; i++)
      {
        double max_width = max(abs(point_caches_.line_width.at(i).left), abs(point_caches_.line_width.at(i).right));
        if (width_ < max_width)
        {
          width_ = max_width;
        }
      }
      return (distance_ < width_);
    }

    RefPoints aim_point;
  };

  // std::map< int, RefLine > ref_line_map; // ref_points.json
  // //  std::vector< RefLine > ref_line_vec;   // ref_line.json appendix_attribute.json ref_line_appendix.json
  // std::map< int, std::vector< RefPoints > > ref_points_map;  // ref_points.json
  // std::map< int, AppendixAttribute > appendix_attribute_map; // ref_points.json
  // std::vector< RefPoints > ref_points_vec;
};

} // namespace map
} // namespace superg_agv
#endif