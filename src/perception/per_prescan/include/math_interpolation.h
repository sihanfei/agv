#ifndef MATH_INTERPOLATION_H_
#define MATH_INTERPOLATION_H_

#include <cstring>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;

namespace superg_agv
{
namespace roi_math
{
struct InterData3D
{
  double data_x;
  double data_y;
  double time;
};

struct InterData2D
{
  double data;
  double time;
};

class MathInterData
{
public:
  int interLagrange(InterData2D &in_b_, InterData2D &in_e_, std::vector< double > &out_time_, int time_count,
                    std::vector< double > &out_data_)
  {
    if (in_e_.time == in_b_.time)
    {
      return -1;
    }

    double s = (in_e_.data - in_b_.data) / (in_e_.time - in_b_.time);

    for (int i = 0; i < time_count; i++)
    {
      double ans = in_e_.data + s * (out_time_.at(i) - in_e_.time);
      out_data_.push_back(ans);
    }

    return 1;
  }

  void interNewton()
  {
  }

  double getDeltaTime(double &time_b, double &time_e, int ave_num)
  {
    return ((time_e - time_b) / (ave_num - 1));
  }

  void getInterValueFrom3D(std::vector< InterData3D > &in_, InterData2D &in_b_x, InterData2D &in_e_x,
                           InterData2D &in_b_y, InterData2D &in_e_y)
  {
    in_b_x.data = in_.begin()->data_x;
    in_b_x.time = in_.begin()->time;
    in_b_y.data = in_.begin()->data_y;
    in_b_y.time = in_.begin()->time;
    in_e_x.data = in_.rbegin()->data_x;
    in_e_x.time = in_.rbegin()->time;
    in_e_y.data = in_.rbegin()->data_y;
    in_e_y.time = in_.rbegin()->time;
  }

  void getOutTimeVec(double time_b, double time_d, int time_count, std::vector< double > &out_time)
  {
    out_time.clear();
    for (int i = 0; i < time_count; i++)
    {
      double time_temp = time_b + time_d * i;
      out_time.push_back(time_temp);
    }
  }

  void getOutVecData3D(int time_count, std::vector< double > &out_x, std::vector< double > &out_y,
                       std::vector< double > &out_time, std::vector< InterData3D > &out_)
  {
    out_.clear();
    for (int i = 0; i < time_count; i++)
    {
      InterData3D out_temp;
      out_temp.data_x = out_x.at(i);
      out_temp.data_y = out_y.at(i);
      out_temp.time   = out_time.at(i);
      out_.push_back(out_temp);
    }
  }

  int mathInterpolation3D(std::vector< InterData3D > &in_, std::vector< InterData3D > &out_, int total_inter_num,
                          int inter_method)
  {
    int count_ = in_.empty() ? -1 : static_cast< int >(in_.size());
    if (count_ < 2 || total_inter_num < 3)
    {
      return -1;
    }

    int ret = 0;
    switch (inter_method)
    {
    case 1:
    {

      InterData2D in_b_x;
      InterData2D in_e_x;
      InterData2D in_b_y;
      InterData2D in_e_y;
      std::vector< double > out_x;
      std::vector< double > out_y;
      std::vector< double > out_time;

      getInterValueFrom3D(in_, in_b_x, in_e_x, in_b_y, in_e_y);

      double time_delta = getDeltaTime(in_b_x.time, in_e_x.time, total_inter_num);

      getOutTimeVec(in_b_x.time, time_delta, total_inter_num, out_time);

      ret = interLagrange(in_b_x, in_e_x, out_time, total_inter_num, out_x);

      if (ret < 0)
      {
        return -1;
      }

      ret = interLagrange(in_b_y, in_e_y, out_time, total_inter_num, out_y);

      if (ret < 0)
      {
        return -1;
      }

      getOutVecData3D(total_inter_num, out_x, out_y, out_time, out_);

      return 1;
      break;
    }

    case 2:
      // interNewton(in_, out_, total_inter_num);
      break;
    default:
        // interLagrange(in_, out_, total_inter_num);
        ;
    }
  }

  int mathInterpolation2D(std::vector< InterData2D > &in_, std::vector< InterData2D > &out_, int total_inter_num,
                          int inter_method)
  {
    return -1;
  }
};
}
}
#endif
